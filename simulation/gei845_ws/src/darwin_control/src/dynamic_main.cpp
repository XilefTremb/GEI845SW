/*

Code for GEI744, ROS and Darwin_Op robotic plateform.
Author: Guillaume Durandau, Guillaume.Durandau@Usherbrooke.ca
License: BSD

Vers: 0.0.1
Date: 05/02/14

*/

// #include "ros/ros.h"
#include "rclcpp/rclcpp.hpp"
#include "builtin_interfaces/msg/duration.hpp"
#include "darwin_control/DarwinJointControl.h"
#include "darwin_control/DarwinReadFile.h"
/*
	joints : j_pelvis_r, j_thigh1_r, j_thigh2_r, j_tibia_r, j_ankle1_r, j_ankle2_r

*/
int main(int argc, char **argv)
{
	rclcpp::init(argc, argv);
 	std::shared_ptr<rclcpp::Node> nh = rclcpp::Node::make_shared("main_node");

	double r = 100.0;  // La fréquence utilisée dans vos calculs
	DarwinJointControl darwin(nh, r);
	boost::thread node_thread(static_cast<void(*)(std::shared_ptr<rclcpp::Node>)>(&rclcpp::spin), nh); // Thread to spin the node.

	DarwinReadFile table("dynamic_test.txt");
	
	darwin.getNameOnScreen();

	int n=2;
	double sizet = table.getTime(0);	
	double temp = table.getTime(1);
	
	rclcpp::Rate time_tot_rate(1.0/5.0);
	rclcpp::Duration time_tot(5,0);
	rclcpp::Rate temps(1.0/temp);

	double theta[12];
	std::string joints[12] = {
		"j_pelvis_r", "j_thigh1_r", "j_thigh2_r", "j_tibia_r", "j_ankle1_r", "j_ankle2_r",
		"j_pelvis_l", "j_thigh1_l", "j_thigh2_l", "j_tibia_l", "j_ankle1_l", "j_ankle2_l"
	};
	
	for (int i=0; i<12; i++)
	{
		theta[i] = table.getJoint(0,i);
		darwin.goalPos(joints[i], theta[i], time_tot);
	}
	time_tot_rate.sleep();

	while (rclcpp::ok())
	{
		for (int i=0; i<12; i++)
		{
        	theta[i] = table.getJoint(n,i);
			darwin.setJoint(joints[i], theta[i]);
		}
        temps.sleep();
        n=n+1;
		if (n >= sizet)
		{
			break;
		}
	}
	return 0;
}
