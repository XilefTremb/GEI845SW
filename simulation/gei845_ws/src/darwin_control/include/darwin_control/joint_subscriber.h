#include "rclcpp/rclcpp.hpp"
#include <fstream>

// **************** Your modifications here **************** //

#include "std_msgs/msg/string.hpp"

class JointSubscriber : public rclcpp::Node
{
  public:
    JointSubscriber(): Node("joint_subscriber")
    {
      fs.open("robot_state.txt");
      subscription_ = this->create_subscription<std_msgs::msg::String>("/topic", 10, std::bind(&JointSubscriber::jointPosCallback, this, std::placeholders::_1));
    }
    ~JointSubscriber()
    {
        fs.close();
    }
  private:
    void jointPosCallback(std_msgs::msg::String::UniquePtr msg);
    std::ofstream fs;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};
// ****************** End of modifications ***************** //
