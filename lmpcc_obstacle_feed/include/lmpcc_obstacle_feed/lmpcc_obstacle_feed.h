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
 *         Boaz, Floor email:
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

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <std_srvs/Empty.h>
#include <lmpcc_msgs/IntTrigger.h>

#include <Eigen/Dense>

#include <vector>

//Dynamic Reconfigure server
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <lmpcc_obstacle_feed/ObstacleFeedConfig.h>

#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>

// custom obstacle messages
#include <lmpcc_msgs/lmpcc_obstacle.h>
#include <lmpcc_msgs/lmpcc_obstacle_array.h>

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// configuration
#include <lmpcc_obstacle_feed/lmpcc_obstacle_feed_configuration.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <spencer_tracking_msgs/TrackedPersons.h>
#include <pedsim_msgs/TrackedPersons.h>

// filter
#include <lmpcc_obstacle_feed/obstacle_prediction.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/LinkStates.h>
#include <people_msgs/People.h>

bool CompareObstacleDistance(lmpcc_msgs::lmpcc_obstacle const &obst1, lmpcc_msgs::lmpcc_obstacle const &obst2);

class ObstacleFeed
{
public:

    ObstacleFeed();

    ~ObstacleFeed();

    bool initialize();

    ros::Subscriber obstacles_sub, optitrack_sub, pedestrians_sub;
    ros::Subscriber state_sub;
    people_msgs::People people_msg_;
    ros::Publisher obstacles_pub, visualize_obstacles_pub, ped_pub_,ped_pub2_, ped_pub3_,ped_pub4_;
    ros::Publisher obst1_path_pub, obst2_path_pub, obst3_path_pub, obst4_path_pub,people_pub;

    std_srvs::Empty emptyCall;
    lmpcc_msgs::IntTrigger IntCall;
    ros::ServiceServer update_service, update_service_int;

    ros::ServiceClient link_state_client_;

    ros::Timer loop_timer;

    boost::shared_ptr<dynamic_reconfigure::Server<lmpcc_obstacle_feed::ObstacleFeedConfig> > reconfigure_server_;
    boost::recursive_mutex reconfig_mutex_;

    void reconfigureCallback(lmpcc_obstacle_feed::ObstacleFeedConfig& config, uint32_t level);

    std::vector<Obstacle_Prediction *> filters_;

    std::vector<ros::Publisher *> link_pubs_;

private:

    ros::NodeHandle nh_;

    tf::TransformListener tf_listener_;

    bool activate_debug_output_;

    boost::shared_ptr<lmpcc_obstacle_feed_configuration> lmpcc_obstacle_feed_config_;

    vision_msgs::Detection3DArray objectArray_;

    double maxV_, minV_, distance_, obstacle_threshold_, obstacle_size_;
    double dt_;
    int N_obstacles_;

    lmpcc_msgs::lmpcc_obstacle_array obstacles_;

    void spinNode();
    void clearDataMember();
    void detectionsCallback(const vision_msgs::Detection3DArray& objects);
    void pedestriansCallback(const spencer_tracking_msgs::TrackedPersons& person);
    void PedsimCallback(const pedsim_msgs::TrackedPersons& person);
    void optitrackCallback(const nav_msgs::Path& predicted_path);
    void updateObstacles(const ros::TimerEvent& event);
    void publishObstacles(const lmpcc_msgs::lmpcc_obstacle_array& obstacles);
    void visualizeObstacles(const lmpcc_msgs::lmpcc_obstacle_array& obstacles);
    void OrderObstacles(lmpcc_msgs::lmpcc_obstacle_array& ellipses);
    bool UpdateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);
    bool UpdateCallbackInt(lmpcc_msgs::IntTrigger::Request& request, lmpcc_msgs::IntTrigger::Response& response);
    bool UpdateCallback();

    lmpcc_msgs::lmpcc_obstacle FitEllipse(const vision_msgs::Detection3D& object, const double& distance);
    lmpcc_msgs::lmpcc_obstacle FitEllipse(const lmpcc_msgs::lmpcc_obstacle& object, const double& distance);
    void QuatToZRot(geometry_msgs::Pose& pose);
    void ZRotToQuat(geometry_msgs::Pose& pose);
    bool getTransform(const std::string& from, const std::string& to, Eigen::VectorXd& transform);
    bool transformPose(const std::string& from, const std::string& to, geometry_msgs::Pose& pose);
    bool transformTwist(const std::string& from, const std::string& to, geometry_msgs::Twist& twist);

};
