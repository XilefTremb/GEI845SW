#ifndef DARWIN_MESS_H
#define DARWIN_MESS_H

// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/Float64.h"
#include "std_msgs/msg/float64.hpp"
// #include "sensor_msgs/JointState.h"
#include "sensor_msgs/msg/joint_state.hpp"
// #include "control_msgs/JointControllerState.h"
#include "control_msgs/msg/JointControllerState.h"
#include <string>
#include <map>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include "darwin_control/DarwinCallback.h"

using namespace ros;
using namespace std;

class DarwinMess
{
public:
	DarwinMess(NodeHandle& node);
private:
	void callback_In(const sensor_msgs::JointState::ConstPtr& msg_in);
	void pub_out(Publisher pub_out, Rate rate);
	map<string, Publisher>			pub_in_;
	Subscriber 						sub_in_;
	Publisher						pub_out_;
	map<string, DarwinCallback*>	sub_out_;
	boost::thread* thread_pub_;
};

#endif
