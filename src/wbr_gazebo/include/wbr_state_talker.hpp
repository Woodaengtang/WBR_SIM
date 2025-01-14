#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using std::placeholders::_1;

class WbrStateTalker : public rclcpp::Node
{
  public:
    WbrStateTalker();

  private:
    void topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
     rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};
