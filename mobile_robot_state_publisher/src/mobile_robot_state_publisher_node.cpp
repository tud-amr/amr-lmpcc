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
 *   ROS package name: mobile_robot_state_publisher
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

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

using namespace std;

double vel;

void VelocityCallBack(const nav_msgs::Odometry& msg){

vel = sqrt(pow(msg.twist.twist.linear.x,2)+pow(msg.twist.twist.linear.y,2));

}

double QuatToJointAngle(const geometry_msgs::TransformStamped transformStamped)
{
    const double ysqr = transformStamped.transform.rotation.y * transformStamped.transform.rotation.y;
    const double t3 = +2.0 * (transformStamped.transform.rotation.w * transformStamped.transform.rotation.z
                 + transformStamped.transform.rotation.x * transformStamped.transform.rotation.y);
    const double t4 = +1.0 - 2.0 * (ysqr + transformStamped.transform.rotation.z * transformStamped.transform.rotation.z);

    return atan2(t3, t4);
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "mobile_robot_state_publisher_node");
	ros::NodeHandle n;
	ros::Subscriber robot_state_sub_;


	double node_rate;
	if (!n.getParam(ros::this_node::getName()+"/rate", node_rate))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/rate not set");
		return 0;
	}

	string root_frame;
	if (!n.getParam(ros::this_node::getName()+"/root_frame", root_frame))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/root_frame not set");
		return 0;
	}

	string base_frame;
	if (!n.getParam(ros::this_node::getName()+"/base_frame", base_frame))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/base_frame not set");
		return 0;
	}

	string robot_state_topic;
	if (!n.getParam(ros::this_node::getName()+"/robot_state_topic", robot_state_topic))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/robot_state_topic not set");
		return 0;
	}

	string vel_state_topic;
	if (!n.getParam(ros::this_node::getName()+"/vel_state_topic", vel_state_topic))
	{
		ROS_ERROR_STREAM("mobile_robot_state_publisher_node Parameter " << ros::this_node::getName()+"/vel_state_topic not set");
		return 0;
	}

    robot_state_sub_ = n.subscribe(vel_state_topic, 1, VelocityCallBack);

	ros::Publisher state_pub_ =
		n.advertise<geometry_msgs::Pose>(robot_state_topic, 10);

	tf2_ros::Buffer tfBuffer;
	tf2_ros::TransformListener tfListener(tfBuffer);

	ros::Rate rate(node_rate);
	geometry_msgs::Pose pose_msg;

	//Intermidiate variables
	double ysqr, t3, t4;
	geometry_msgs::TransformStamped transformStamped;
	while (n.ok()){
		try{
			transformStamped = tfBuffer.lookupTransform(root_frame, base_frame,
														ros::Time(0));
		}
		catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		pose_msg.orientation.z = QuatToJointAngle(transformStamped);

		pose_msg.position.x = transformStamped.transform.translation.x;
		pose_msg.position.y = transformStamped.transform.translation.y;
		pose_msg.position.z = vel;
		state_pub_.publish(pose_msg);

		ros::spinOnce();
	}

	return 0;
}
