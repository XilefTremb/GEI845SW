/*

Code for GEI744, ROS and Darwin_Op robotic plateform.
Author: Guillaume Durandau, Guillaume.Durandau@Usherbrooke.ca
License: BSD

Vers: 0.0.1
Date: 04/02/14

*/

#ifndef DARWINJOINT_H
#define DARWINJOINT_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <string>
#include <boost/thread/thread.hpp>
// #include <boost/thread/mutex.hpp>
#include <cmath>
// #include <memory>

using namespace std;

class DarwinJoint// : public rclcpp::Node
{
public:
	DarwinJoint(std::string joint_name, std::shared_ptr<rclcpp::Node> node, double rate);
	~DarwinJoint();
	double getJoint();
	std::string getName();
	void setJoint(double joint_send_value);
	bool initEnd();
	void goalPos(double j_val, rclcpp::Duration tf);
private:
	void darwinCallback(sensor_msgs::msg::JointState::SharedPtr msg_in); /* Callback function for the subscriber */
	void pubThread(); /* Thread function */
	void goalPosThread(double j_fin, rclcpp::Duration tf);
	// boost::mutex mutex_; /* Mutex for thread */

	std::shared_ptr<rclcpp::Node> node_;
	double send_value_; /* Value for thread and main */
	boost::thread* thread_pub_; /* Thread for the publisher TODO: supprimer thread pour publisher et l'appller adns la classe supp*/
	boost::thread* thread_goal_; /* Thread for the goal position function */
	rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr joint_pub_;
	rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
	std_msgs::msg::Float64MultiArray msg_out_; /* Message for the publisher in the thread */
	std::string darwin_joint_name_; /* Name of the joint in this class */
	double joint_value_; /* Value of the joint */
	bool stop_thread_; /* Bool for kill the thread */
	bool first_msg;
	double rate_;
};

#endif
