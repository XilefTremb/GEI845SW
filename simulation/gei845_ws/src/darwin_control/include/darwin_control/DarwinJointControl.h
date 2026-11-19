/*

Code for GEI744, ROS and Darwin_Op robotic plateform.
Author: Guillaume Durandau, Guillaume.Durandau@Usherbrooke.ca
License: BSD

Vers: 0.0.1
Date: 04/02/14

*/

#ifndef DARWINJOINTCONTROL_H
#define DARWINJOINTCONTROL_H

// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
// #include "std_msgs/Float64.h"
#include "std_msgs/msg/float64.hpp"
#include <string>
#include <map>
#include "darwin_control/DarwinJoint.h"


class DarwinJointControl// : public rclcpp::Node
{
public:
	DarwinJointControl(std::shared_ptr<rclcpp::Node> node, double rate);
	double getJoint(std::string joint_name);
	void setJoint(std::string joint_name, double joint_send_value);
	void getNameOnScreen();
	void goalPos(std::string joint_name, double joint_send_value, rclcpp::Duration time_tot);
private:
	std::shared_ptr<rclcpp::Node> node_;
	double rate_;
	std::map<std::string, DarwinJoint*> joint_map_;
};

#endif
