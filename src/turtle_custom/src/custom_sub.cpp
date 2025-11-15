#include <functional>
#include <memory>
#include <thread>
#include <vector>
#include <sstream>
#include "turtle_custom/custom_sub.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace custom
{

    static const float MAX_STEP_LENGTH = 1;
    static const float MAX_STEP_ANG = 30;

    TurtlePlugin::TurtlePlugin(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
        : Node("turtle_plugin", options), was_out_(false)
    {
        std::string turtle_name = "turtle1";
        float margin = 0.5;

        // Initial command message
        cmd_vel_msg_.linear.x = 1.3;
        cmd_vel_msg_.angular.z = -0.5;

        // Initialize boundaries
        tl.x = margin;
        tl.y = margin;
        br.x = 11.11 - margin;
        br.y = 11.11 - margin;

        turtlesim_client_ = std::make_shared<rclcpp::AsyncParametersClient>(this, "sim");

        /* Publisher on /cmd_vel */
        cmd_vel_pub_ =
            this->create_publisher<Twist>(turtle_name + "/cmd_vel", 10);
        auto timer_callback = [this]() -> void
        {
            this->cmd_vel_pub_->publish(cmd_vel_msg_);
        };
        timer_ = this->create_wall_timer(500ms, timer_callback);

        /* Subsription do /pose */
        auto pose_callback =
            [this](Pose::UniquePtr msg) -> void
        {
            cur_pose_ = *msg;

            /* Background color switch */
            bool is_out = is_out_(cur_pose_);
            if (is_out != was_out_)
            {
                if (is_out)
                    turtlesim_client_->set_parameters(
                        {rclcpp::Parameter("background_r", 122),
                         rclcpp::Parameter("background_g", 42),
                         rclcpp::Parameter("background_b", 42)});
                else
                    turtlesim_client_->set_parameters(
                        {rclcpp::Parameter("background_r", 0),
                         rclcpp::Parameter("background_g", 122),
                         rclcpp::Parameter("background_b", 84)});
                was_out_ = is_out;
            }
        };
        pose_sub_ = this->create_subscription<Pose>(turtle_name + "/pose", 10, pose_callback);

        /* Pen toggle parameter */
        this->declare_parameter("is_pen_off", false);
        pen_toggle_sub_ = std::make_shared<rclcpp::ParameterEventHandler>(this);
        pen_client_ = this->create_client<SetPen>(turtle_name + "/set_pen");
        pen_request_ = std::make_shared<SetPen::Request>();
        pen_request_->width = 3;
        pen_request_->r = 180;
        pen_request_->g = 180;
        pen_request_->b = 255;

        auto cb = [this](const rclcpp::Parameter &p)
        {
            pen_request_->off = p.as_bool();
            auto result_future = pen_client_->async_send_request(pen_request_);
            cmd_vel_msg_.angular.z = -cmd_vel_msg_.angular.z;
        };
        pen_toggle_cb_ = pen_toggle_sub_->add_parameter_callback("is_pen_off", cb);

        /* Trajectory generation action */
        using namespace std::placeholders;

        this->declare_parameter("k_linear", 1.0);
        this->declare_parameter("k_angular", 3.0);

        auto handle_goal = [this](
                               const rclcpp_action::GoalUUID &uuid,
                               std::shared_ptr<const TrajGen::Goal> goal)
        {
            RCLCPP_INFO(this->get_logger(), "Received goal request");
            int n_points = goal->path.size();
            if (n_points == 0)
            {
                RCLCPP_WARN(this->get_logger(), "Path cannot be empty");
                return rclcpp_action::GoalResponse::REJECT;
            }
            for (int i = 0; i < n_points; i++)
            {
                if (is_out_(goal->path[i]))
                {
                    RCLCPP_WARN(this->get_logger(), "Target points must be accessible");
                    return rclcpp_action::GoalResponse::REJECT;
                }
            }
            (void)uuid;
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        };

        auto handle_cancel = [this](
                                 const std::shared_ptr<GoalHandleTrajGen> goal_handle)
        {
            RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
            if (goal_handle->is_canceling())
            {
                cmd_vel_msg_.linear.x = 0.0;
                cmd_vel_msg_.angular.z = 0.0;
            }
            return rclcpp_action::CancelResponse::ACCEPT;
        };

        auto handle_accepted = [this](
                                   const std::shared_ptr<GoalHandleTrajGen> goal_handle)
        {
            auto execute_in_thread = [this, goal_handle]()
            { return this->execute(goal_handle); };
            std::thread{execute_in_thread}.detach();
        };

        this->traj_gen_server_ = rclcpp_action::create_server<TrajGen>(
            this,
            "traj_gen",
            handle_goal,
            handle_cancel,
            handle_accepted);
    }

    bool TurtlePlugin::is_out_(const Pose &p)
    {
        return ((p.x < tl.x ||
                 p.x > br.x ||
                 p.y < tl.y ||
                 p.y > br.y));
    }

    void TurtlePlugin::execute(const std::shared_ptr<GoalHandleTrajGen> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
        rclcpp::Rate loop_rate(10);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<TrajGen::Feedback>();
        auto &sequence = feedback->next_target;
        // sequence.push_back(0);
        // sequence.push_back(1);
        auto result = std::make_shared<TrajGen::Result>();

        std::vector<Pose> traj_planned;
        interpolate(traj_planned, cur_pose_, goal->path[0]);
        for (u_long i = 1; i < goal->path.size(); i++)
        {
            interpolate(traj_planned, goal->path[i - 1], goal->path[i]);
        }

        for (size_t i = 1; i < traj_planned.size() && rclcpp::ok();)
        {
            if (goal_handle->is_canceling())
            {
                result->path_size = i;
                goal_handle->canceled(result);
                return;
            }

            Pose target = traj_planned[i];

            // Commande vers ce waypoint
            control_towards(target);

            // Vérifier distance d’arrivée
            double dx = target.x - cur_pose_.x;
            double dy = target.y - cur_pose_.y;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < 0.05) // seuil
            {
                auto feedback = std::make_shared<TrajGen::Feedback>();
                feedback->next_target = target;
                goal_handle->publish_feedback(feedback);

                RCLCPP_INFO(this->get_logger(),
                            "Reached waypoint %zu at (%.2f, %.2f)",
                            i, target.x, target.y);

                i++; // passer au waypoint suivant
            }

            loop_rate.sleep();
        }

        // Check if goal is done
        if (rclcpp::ok())
        {
            // result->path_size = i;
            goal_handle->succeed(result);
            cmd_vel_msg_.linear.x = 0.0;
            cmd_vel_msg_.angular.z = 0.0;

            RCLCPP_INFO(this->get_logger(), "Goal succeeded");
        }
    }

    void TurtlePlugin::interpolate(std::vector<Pose> &traj, const Pose &start, const Pose &end)
    {
        float Dx = end.x - start.x;
        float Dy = end.y - start.y;

        float Dl = std::hypot(Dx, Dy);

        uint n_step_pos = ceil(Dl / MAX_STEP_LENGTH);

        float dx = (end.x - start.x) / n_step_pos;
        float dy = (end.y - start.y) / n_step_pos;

        for (uint j = 0; j < n_step_pos; j++)
        {
            Pose p;
            p.x = start.x + dx * j;
            p.y = start.y + dy * j;
            p.theta = end.theta;
            traj.push_back(p);
        }
        traj.push_back(end);
    }

    void TurtlePlugin::control_towards(const Pose &target)
    {
        double dx = target.x - cur_pose_.x;
        double dy = target.y - cur_pose_.y;

        double dist = std::sqrt(dx * dx + dy * dy);
        double angle = std::atan2(dy, dx);

        double dtheta = angle - cur_pose_.theta;
        while (dtheta > M_PI)
            dtheta -= 2 * M_PI;
        while (dtheta < -M_PI)
            dtheta += 2 * M_PI;

        double k_l = this->get_parameter("k_linear").as_double();
        double k_a = this->get_parameter("k_angular").as_double();

        cmd_vel_msg_.linear.x = k_l * dist;
        cmd_vel_msg_.angular.z = k_a * dtheta;
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(custom::TurtlePlugin);