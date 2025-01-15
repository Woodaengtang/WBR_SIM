#include "wbr_state_talker.hpp"

WbrStateTalker::WbrStateTalker() : Node("wbr_state_talker") {
  subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10, std::bind(&WbrStateTalker::topic_callback, this, _1));
}

void WbrStateTalker::topic_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  std::cout << std::endl << std::endl << std::endl;
  std::cout << "B2TAR pos : " << std::to_string(msg->position[0]) << std::endl;
  std::cout << "B2TAL pos : " << std::to_string(msg->position[1]) << std::endl;
  std::cout << "CR2WR pos : " << std::to_string(msg->position[2]) << std::endl;
  std::cout << "CL2WL pos : " << std::to_string(msg->position[3]) << std::endl;
  std::cout << "B2TAR vel : " << std::to_string(msg->velocity[0]) << std::endl;
  std::cout << "B2TAL vel : " << std::to_string(msg->velocity[1]) << std::endl;
  std::cout << "CR2WR vel : " << std::to_string(msg->velocity[2]) << std::endl;
  std::cout << "CL2WL vel : " << std::to_string(msg->velocity[3]) << std::endl;
  // RCLCPP_INFO(this->get_logger(), "I heard position[0]: %f", msg->position[0]);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WbrStateTalker>());
  rclcpp::shutdown();
  return 0;
}
