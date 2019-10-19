/*
 /*!
 *****************************************************************
 * \file
 *
 * \note
 * Copyright (c) 2019 \n
 * TU DELFT \n\n
 *
 *****************************************************************
 *
 * \note
 * ROS stack name: arm-lmpcc
 * \note
 * ROS package name: lmpcc
 *
 * \author
 * Authors: Bruno Brito   email: bruno.debrito@tudelft.nl
 *          Boaz, Floor email: boazfloor@gmail.com
 *
 * \date Date of creation: June, 2019
 *
 * \brief
 *
 * *****************************************************************
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer. \n
 * - Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution. \n
 * - Neither the name of the TU Delft nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission. \n
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License LGPL as
 * published by the Free Software Foundation, either version 3 of the
 * License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License LGPL for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License LGPL along with this program.
 * If not, see <http://www.gnu.org/licenses/>.
 *
 ******************************************************************/

#pragma once

// ros includes
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <lmpcc_msgs/IntTrigger.h>

// eigen includes
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

// Include pre-generated OCP
#include <acado_common.h>
#include <acado_auxiliary_functions.h>

// std includes
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <limits>

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// Visualization messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

// yaml parsing
#include <fstream>
#include <yaml-cpp/yaml.h>

// lmpcc configuration
#include <lmpcc/lmpcc_configuration.h>

// reference path class
#include <lmpcc/reference_path.h>

// actions, srvs, msgs
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
//#include <lmpcc/collision_avoidance.h>

// joint trajectory interface
#include <control_msgs/FollowJointTrajectoryAction.h>

// navigation messages
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/GetMap.h>

//#include <costmap_2d/costmap_2d_ros.h>
//#include <costmap_2d/costmap_2d.h>

// lmpcc messages
#include <lmpcc_msgs/lmpcc_feedback.h>
#include <lmpcc_msgs/lmpcc_obstacle.h>
#include <lmpcc_msgs/lmpcc_obstacle_array.h>

//Dynamic Reconfigure server
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <lmpcc/LmpccConfig.h>

//TF
#include <tf2_ros/transform_broadcaster.h>

//Joint states
#include <sensor_msgs/JointState.h>

//splines
#include <tkspline/spline.h>
#include <lmpcc/Clothoid.h>
#include <static_collision_avoidance/collision_free_polygon.h>

//reset msgs
#include <std_srvs/Empty.h>
#include <robot_localization/SetPose.h>
#include <chrono>

typedef double real_t;

class LMPCC
{
    /** Managing execution of all classes of lmpcc
     * - Handle self collision avoidance
     * - Extract current position and velocity of manipulator joints
     * - Publish controlled joint velocity
     */
public:

    // Dynamic reconfigure server
    boost::shared_ptr<dynamic_reconfigure::Server<lmpcc::LmpccConfig> > reconfigure_server_;
    boost::recursive_mutex reconfig_mutex_;
    void reconfigureCallback(lmpcc::LmpccConfig& config, uint32_t level);

    /**
     * @brief LMPCC: Default constructor, allocate memory
     */
    LMPCC()
    {
        this->reconfigure_server_.reset();      // Reset reconfigure server
    };

    /**
     * @brief ~LMPCC: Default destructor, free memory
     */
    ~LMPCC();

    /**
     * @brief initialize: Initialize all helper class of lmpcc control and initialize ros subscribers and publishers
     * @return: Returns true if initialized successfully, false otherwise
     */
    bool initialize();


    bool initialize_visuals();

    /**
     * @brief StateCallBack: Get current state of the robot
     * @param msg: Read data from mobile_robot_state_publisher_node default type:
     */
    void StateCallBack(const geometry_msgs::Pose::ConstPtr& msg);

    /**
     * @brief ObstacleCallBack: Get current state of moving obstacles
     * @param obstacles: Data containing the current moving obstacle configurations
     */
    void ObstacleCallBack(const lmpcc_msgs::lmpcc_obstacle_array& received_obstacles);

    void FreeAreaCallBack(const static_collision_avoidance::collision_free_polygon& msg);

    /**
     * @brief getTransform: Find transformation stamed rotation is in the form of quaternion
     * @param from: source frame from find transformation
     * @param to: target frame till find transformation
     * @param stamped_pose: Resultant poseStamed between source and target frame
     * @return: true if transform else false
     */
    bool getTransform(const std::string& from,
                                        const std::string& to,
                                        geometry_msgs::PoseStamped& stamped_pose
                                        );

    bool transformPose(const std::string& from, const std::string& to, geometry_msgs::Pose& pose);

    /**
     * @brief transformStdVectorToEigenVector: tranform std vector to eigen vectors as std vectos are slow to random access
     * @param vector: std vectors want to tranfrom
     * @return Eigen vectors transform from std vectos
     */
    template<typename T>
    static inline Eigen::VectorXd transformStdVectorToEigenVector(const std::vector<T>& vector)
    {
        // resize eigen vector
        Eigen::VectorXd eigen_vector = Eigen::VectorXd(vector.size());

        // convert std to eigen vector
        for (uint32_t i = 0; i < vector.size(); ++i)
        {
            eigen_vector(i) = vector.at(i);
        }

        return eigen_vector;
    }

    double spline_closest_point(double s_min, double s_max, double s_guess, double window, int n_tries);

    void computeContourError(void);

    void  reset_solver();

    //Service clients
    ros::ServiceClient reset_simulation_client_,reset_ekf_client_;
    //reset simulation msg
    std_srvs::Empty reset_msg_;
    robot_localization::SetPose reset_pose_msg_;

    /** public data members */

    ros::ServiceClient update_trigger;

    // joint state subsciber to get current joint value
    ros::Subscriber robot_state_sub_;

    // subscriber for obstacle feed
    ros::Subscriber obstacle_feed_sub_;

    // subscriber for pedestrian feed
    ros::Subscriber pedestrian_feed_sub_,collision_free_sub_;

    // controlled joint velocity, should be control velocity of controller
    ros::Publisher controlled_velocity_pub_;

    // publish trajectory
    ros::Publisher traj_pub_, tr_path_pub_, pred_traj_pub_, pred_cmd_pub_,cost_pub_,robot_collision_space_pub_, global_plan_pub_,local_spline_traj_pub1_, local_spline_traj_pub2_, local_spline_traj_pub3_, local_spline_traj_pub4_, local_spline_traj_pub5_, contour_error_pub_, feedback_pub_;
	//Predicted trajectory
	nav_msgs::Path pred_traj_;
	nav_msgs::Path pred_cmd_;
	nav_msgs::Path local_spline_traj1_,local_spline_traj2_,local_spline_traj3_,local_spline_traj4_,local_spline_traj5_;

    lmpcc_msgs::IntTrigger obstacle_trigger;

	int segment_counter;

	//Controller options
    real_t te_, te_collision_free_;

	tf2_ros::TransformBroadcaster state_pub_, path_pose_pub_;
	std_msgs::Float64 cost_;
    double contour_error_;
    double lag_error_;
    int n_search_points_;
    int n_obstacles_;

    std::string cmd_topic_;

	//Spline trajectory generation
    std::vector<double> X_road, Y_road, Theta_road;
    double dist_spline_pts_;
    double total_length_;
    double path_length_;
    std::vector<double> ss,xx,yy,vv;
	tk::spline ref_path_x, ref_path_y;
    int n_clothoid_,n_pts_;

    std::chrono::steady_clock::time_point ini_t_,end_t_;

    //Search window parameters
    bool goal_reached_;
    bool plan_;
    std::vector<double> collision_free_C1, collision_free_C2, collision_free_C3, collision_free_C4, collision_free_a1x ,collision_free_a1y, collision_free_a2x ,collision_free_a2y, collision_free_a3x ,collision_free_a3y, collision_free_a4x ,collision_free_a4y , collision_free_xmin, collision_free_xmax, collision_free_ymin, collision_free_ymax;

    ReferencePath referencePath;

private:
    /**
     * @brief spinNode: spin node means ROS is still running
     */
    void spinNode();

    void computeEgoDiscs();

    /**
     * @brief controlLoop: Continue updating this function depend on controller frequency
     * @param event: Used for computation of duration of first and last event
     */
    void controlLoop(const ros::TimerEvent& event);

    /**
     * @brief publishZeroJointVelocity: published zero joint velocity is statisfied cartesian distance
     */
    void publishZeroJointVelocity();

    void publishTrajectory(void);

	/**
	 * @brief publishPredictedTrajectory: publish predicted trajectory
	 */
	void publishPredictedTrajectory(void);

	void publishGlobalPlan(void);

	void publishLocalRefPath(void);

	void publishPredictedOutput(void);

	void publishPredictedCollisionSpace(void);

	void publishCost(void);

    void publishContourError(void);

	void broadcastTF();

	void broadcastPathPose();

    void ZRotToQuat(geometry_msgs::Pose& pose);

    void publishFeedback(int& it, double& time);

    ros::NodeHandle nh;

    tf::TransformListener tf_listener_;

    double r_discs_;
    Eigen::VectorXd x_discs_;

    // Timer
    ros::Timer timer_;

    // used to set desired position by mannually or using interactive marker node
    bool tracking_;
    std::string target_frame_;

    visualization_msgs::MarkerArray traj_marker_array_;

    Eigen::Vector4d current_state_, last_state_;

    visualization_msgs::Marker ellips1, global_plan;

    // Obstacles
    lmpcc_msgs::lmpcc_obstacle_array obstacles_;
    lmpcc_msgs::lmpcc_obstacle_array obstacles_init_;

    // Type of variable used to publish joint velocity
    geometry_msgs::Twist controlled_velocity_;

    // lmpcc configuration
    boost::shared_ptr<LMPCC_configuration> lmpcc_config_;

    Eigen::VectorXd cost_contour_weight_factors_;
    Eigen::VectorXd cost_control_weight_factors_;

    double slack_weight_;
    double repulsive_weight_;

    double reference_velocity_,reduced_reference_velocity_;

    bool enable_output_;
    bool loop_mode_;
    int n_iterations_;

    int n_traj_per_cloth;
    double window_size_;

};
