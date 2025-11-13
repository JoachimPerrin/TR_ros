#ifndef TURTLE_CUSTOM_HPP
#define TURTLE_CUSTOM_HPP

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/vector3.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtle_action_interfaces/action/traj_gen.hpp"

#include "turtle_custom/visibility_control.h"


namespace custom
{
using namespace geometry_msgs::msg;
using namespace turtlesim::msg;
using namespace turtlesim::srv;
using namespace turtle_action_interfaces::action;
using namespace std::chrono_literals;
using GoalHandleTrajGen = rclcpp_action::ServerGoalHandle<TrajGen>;

class TurtlePlugin : public rclcpp::Node
{
public:
    TURTLE_CUSTOM_CPP_PUBLIC
    explicit TurtlePlugin(const rclcpp::NodeOptions &options);

private:
    bool is_out_(const Pose &p);
    void execute(const std::shared_ptr<GoalHandleTrajGen> goal_handle);

    Twist cmd_vel_msg_;
    Pose cur_pose_;
    std::shared_ptr<SetPen::Request> pen_request_;
    Pose tl, br;
    bool was_out_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Subscription<Pose>::SharedPtr pose_sub_;

    std::shared_ptr<rclcpp::ParameterEventHandler> pen_toggle_sub_;
    std::shared_ptr<rclcpp::ParameterCallbackHandle> pen_toggle_cb_;

    rclcpp::Client<SetPen>::SharedPtr pen_client_;
    rclcpp::AsyncParametersClient::SharedPtr turtlesim_client_;

    rclcpp_action::Server<TrajGen>::SharedPtr traj_gen_server_;
};
}
#endif