#include "wbr_state_talker.hpp"

WbrStateTalker::WbrStateTalker() : Node("wbr_state_talker") {
  subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&WbrStateTalker::topic_callback, this, _1));
}

void WbrStateTalker::topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "I heard position[0]: %f", msg->position[0]);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WbrStateTalker>());
  rclcpp::shutdown();
  return 0;
}
