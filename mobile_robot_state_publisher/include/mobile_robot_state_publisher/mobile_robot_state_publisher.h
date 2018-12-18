/*!
 *****************************************************************
 * \file
 *
 * \note
 *   Copyright (c) 2018 \n
 *   TU Delft
 *
 *****************************************************************
 *
 * \note
 *   Project name:
 * \note
 *   ROS stack name:
 * \note
 *   ROS package name: mobile_robot_stsate_publisher
 *
 * \author
 *   Author: Bruno Brito, email: Bruno.deBrito@tudelft.nl
 *
 * \date Date of creation: May, 2018
 *
 * \brief
 *   This package provides a generic mobile_robot_stsate_publisher
 *
 ****************************************************************/


#ifndef PROJECT_MOBILE_ROBOT_STATE_PUBLISHER_H
#define PROJECT_MOBILE_ROBOT_STATE_PUBLISHER_H

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>


class MobileRobotStatePublisher
{
	private:
	ros::NodeHandle nh_;

	ros::Subscriber jointstate_sub_;

	ros::Publisher robot_state_pub_;

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	public:
	MobileRobotStatePublisher()
	{
	}

	~MobileRobotStatePublisher()
	{

	}

	bool initialize();

};

#endif //PROJECT_MOBILE_ROBOT_STATE_PUBLISHER_H