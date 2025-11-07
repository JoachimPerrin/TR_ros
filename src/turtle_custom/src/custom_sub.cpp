#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
// #include "geometry_msgs/msg/vector3.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/msg/pose.hpp"

using geometry_msgs::msg::Twist;
// using geometry_msgs::msg::Vector3;
using turtlesim::msg::Pose;
using turtlesim::srv::SetPen;

class PositionMonitor : public rclcpp::Node
{
public:

  PositionMonitor()
  : Node("movement_subscriber"),
    was_out(false)
  {
    bottom_left.x = 1.0f;
    bottom_left.y = 1.0f;

    top_right.x = 10.0f;
    top_right.y = 10.0f;
    
    // Subscription to turtle Pose
    auto pose_callback =
      [this](Pose::UniquePtr msg) -> void {
        bool out = ((msg->x < bottom_left.x) ||
                    (msg->x > top_right.x) ||
                    (msg->y < bottom_left.y) ||
                    (msg->y > top_right.y));

        if(this->was_out != out)
        {
          if(out)
          {
            set_parameter("background_r", 255);
            set_parameter("background_g", 0);
            set_parameter("background_b", 0);
          }
          else
          {
            set_parameter("background_r", 0);
            set_parameter("background_g", 255);
            set_parameter("background_b", 0);
          }
        }
        this->was_out = out;
      };
    pose_sub_ = this->create_subscription<Pose>("/turtle1/pose", 10, pose_callback);


    pen_request_ = std::make_shared<SetPen::Request>();
    pen_request_->off = true;

    param_subscriber_ = std::make_shared<rclcpp::ParameterEventHandler>(this);  
    this->declare_parameter("pen_bool_param", false);
    auto toggle_pen_cb = [this](const rclcpp::Parameter & p) { pen_request->off = !pen_request->off; };
    cb_pen_handle_ = pen_param_sub_->add_parameter_callback("pen_bool_param", toggle_pen_cb);
  }
  
private:

  rclcpp::Subscription<Pose>::SharedPtr pose_sub_;

  std::shared_ptr<SetPen> pen_request_;
  std::shared_ptr<rclcpp::ParameterEventHandler> pen_param_sub_;
  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_pen_handle_;

  Pose bottom_left, top_right;
  bool was_out;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PositionMonitor>());
  rclcpp::shutdown();
  return 0;
}