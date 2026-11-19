#ifndef DARWIN_CALLBACK_H
#define DARWIN_CALLBACK_H

// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/Float64.h"
#include "std_msgs/msg/float64.hpp"
// #include "sensor_msgs/JointState.h"
#include "sensor_msgs/msg/joint_state.hpp"
#include <string>
#include <map>
#include <boost/thread/mutex.hpp>

// using namespace ros;
// using namespace std;

typedef struct msg_struct {
    map<string, double> data;
    boost::mutex mutex_;
} msg_struct;

class DarwinCallback
{
public:
	DarwinCallback(rclcpp::Node node, string name_joint);
	void callback(const std_msgs::msg::Float64::ConstPtr& msg_in);
	static msg_struct msg_data;
private:
	//Publisher pub_;
	Subscriber sub_;
	string name_joint_;
};

#endif
