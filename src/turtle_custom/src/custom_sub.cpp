#include <memory>
#include "turtle_custom/custom_sub.hpp"
#include "rclcpp_components/register_node_macro.hpp"

TURTLE_CUSTOM_CPP_PUBLIC
explicit PositionMonitor::PositionMonitor(const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
    : Node("movement_subscriber", options), was_out_(false)
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

    auto handle_goal = [this](
                           const rclcpp_action::GoalUUID &uuid,
                           std::shared_ptr<const TrajGen::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->targets[0].x);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    };

    auto handle_cancel = [this](
                             const std::shared_ptr<GoalHandleTrajGen> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    };

    auto handle_accepted = [this](
                               const std::shared_ptr<GoalHandleTrajGen> goal_handle)
    {
        // this needs to return quickly to avoid blocking the executor,
        // so we declare a lambda function to be called inside a new thread
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

bool PositionMonitor::is_out_(const Pose &p)
{
    return ((p.x < tl.x ||
             p.x > br.x ||
             p.y < tl.y ||
             p.y > br.y));
}

void PositionMonitor::execute(const std::shared_ptr<GoalHandleTrajGen> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(10);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<TrajGen::Feedback>();
    auto &sequence = feedback->path_taken;
    // sequence.push_back(0);
    // sequence.push_back(1);
    auto result = std::make_shared<TrajGen::Result>();

    for (int i = 1; (i < goal->order) && rclcpp::ok(); ++i)
    {
        // Check if there is a cancel request
        if (goal_handle->is_canceling())
        {
            result->sequence = sequence;
            goal_handle->canceled(result);
            RCLCPP_INFO(this->get_logger(), "Goal canceled");
            return;
        }
        // Update sequence
        sequence.push_back(sequence[i] + sequence[i - 1]);
        // Publish feedback
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "Publish feedback");

        loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok())
    {
        result->sequence = sequence;
        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(turtle_custom::TrajGenServer);

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionMonitor>());
    rclcpp::shutdown();
    return 0;
}