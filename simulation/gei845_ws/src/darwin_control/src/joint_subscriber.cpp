#include "darwin_control/joint_subscriber.h"

// **************** Your modifications here **************** //
void JointSubscriber::jointPosCallback(std_msgs::msg::String::UniquePtr msg) 
{
  RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
};
// ****************** End of modifications ***************** //

int main(int argc, char **argv)
{

  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<JointSubscriber>());
  rclcpp::shutdown();

  return 0;
}
