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

#include <lmpcc_obstacle_feed/lmpcc_obstacle_feed.h>

ObstacleFeed::ObstacleFeed()
{
    this->reconfigure_server_.reset();
}

ObstacleFeed::~ObstacleFeed()
{
  clearDataMember();
}

constexpr double DEFAULT_OBSTACLE_DISTANCE = 1000;
constexpr double DEFAULT_OBSTACLE_SIZE = 0.01;

bool ObstacleFeed::initialize()
{
    if (ros::ok())
    {
        // initialize parameter configuration class
        lmpcc_obstacle_feed_config_.reset(new lmpcc_obstacle_feed_configuration());

        bool lmpcc_obstacle_feed_config_success = lmpcc_obstacle_feed_config_->initialize();
        if (lmpcc_obstacle_feed_config_success == false)
        {
            ROS_ERROR("OBSTACLE FEED: FAILED TO INITIALIZE!!");
            return false;
        }

        /** Initialize reconfigurable parameters **/
        minV_ = lmpcc_obstacle_feed_config_->min_obstacle_volume_;
        maxV_ = lmpcc_obstacle_feed_config_->max_obstacle_volume_;
        N_obstacles_ = lmpcc_obstacle_feed_config_->obstacle_number_;
        distance_ = lmpcc_obstacle_feed_config_->distance_threshold_;
        obstacle_size_ = lmpcc_obstacle_feed_config_->obstacle_size_;

        dt_ = lmpcc_obstacle_feed_config_->prediction_horizon_/lmpcc_obstacle_feed_config_->discretization_steps_;

        /** Initialize publisher of ellipsoid obstacles **/
        obstacles_pub = nh_.advertise<lmpcc_msgs::lmpcc_obstacle_array>(lmpcc_obstacle_feed_config_->pub_obstacles_,1);
        people_pub = nh_.advertise<people_msgs::People>(lmpcc_obstacle_feed_config_->pub_people_,1);
        people_msg_.people.resize(lmpcc_obstacle_feed_config_->obstacle_number_);
        people_msg_.header.frame_id="map";

        /** Pedstrian twist publisher **/
        link_pubs_.resize(lmpcc_obstacle_feed_config_->obstacle_number_);
        for (int obst_it = 1; obst_it < lmpcc_obstacle_feed_config_->obstacle_number_+1; obst_it++){
            link_pubs_[obst_it-1] = new ros::Publisher(nh_.advertise<geometry_msgs::PoseStamped>("ped_link_"+std::to_string(obst_it),1));
        }

        /** Initialize obstacle visualization **/
        if (lmpcc_obstacle_feed_config_->activate_visualization_) {
            visualize_obstacles_pub = nh_.advertise<visualization_msgs::MarkerArray>(lmpcc_obstacle_feed_config_->pub_obstacles_vis_,1);
            obst1_path_pub = nh_.advertise<nav_msgs::Path>("obst1_path",1);
            obst2_path_pub = nh_.advertise<nav_msgs::Path>("obst2_path",1);
            obst3_path_pub = nh_.advertise<nav_msgs::Path>("obst3_path",1);
            obst4_path_pub = nh_.advertise<nav_msgs::Path>("obst4_path",1);
        }

        /** Setting up dynamic_reconfigure server for the ObstacleFeedConfig parameters **/
        ros::NodeHandle nh_obstacle("lmpcc_obstacle_feed");
        reconfigure_server_.reset(new dynamic_reconfigure::Server<lmpcc_obstacle_feed::ObstacleFeedConfig>(reconfig_mutex_, nh_obstacle));
        reconfigure_server_->setCallback(boost::bind(&ObstacleFeed::reconfigureCallback,this,_1,_2));

        // setting resize value
        obstacles_.lmpcc_obstacles.resize(lmpcc_obstacle_feed_config_->obstacle_number_);  //resize according to number of obstacles
        obstacles_.header.stamp = ros::Time::now();
        obstacles_.header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;

        // resize filter
        filters_.resize(lmpcc_obstacle_feed_config_->obstacle_number_);
        double dt = 1/lmpcc_obstacle_feed_config_->prediction_horizon_;
        for (int obst_it = 0; obst_it < obstacles_.lmpcc_obstacles.size(); obst_it++)
        {
            filters_[obst_it] =  new Obstacle_Prediction(dt,lmpcc_obstacle_feed_config_->discretization_steps_);
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
            obstacles_.lmpcc_obstacles[obst_it].trajectory.header.stamp = ros::Time::now();
            obstacles_.lmpcc_obstacles[obst_it].trajectory.header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;

            obstacles_.lmpcc_obstacles[obst_it].pose.position.x = DEFAULT_OBSTACLE_DISTANCE ;  //initializing obstacles
            obstacles_.lmpcc_obstacles[obst_it].pose.position.y = DEFAULT_OBSTACLE_DISTANCE;
            obstacles_.lmpcc_obstacles[obst_it].pose.orientation.z = 0;

            obstacles_.lmpcc_obstacles[obst_it].minor_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_); //resize according to time horizon
            obstacles_.lmpcc_obstacles[obst_it].major_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);

            obstacles_.lmpcc_obstacles[obst_it].minor_semiaxis[0] = DEFAULT_OBSTACLE_SIZE;
            obstacles_.lmpcc_obstacles[obst_it].major_semiaxis[0] = DEFAULT_OBSTACLE_SIZE;


            for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++)
            {
                obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.stamp = ros::Time::now();
                obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;
                obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.x = DEFAULT_OBSTACLE_DISTANCE;
                obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.y = DEFAULT_OBSTACLE_DISTANCE;

                obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.orientation.z = 0;
            }
        }

        /** Initialize appropriate variables according to operation mode **/
        if (lmpcc_obstacle_feed_config_->obstacle_feed_mode_ == 2)
        {
            ROS_WARN("In this mode, predefined obstacles are published");

            int update_rate = lmpcc_obstacle_feed_config_->update_rate_;

            loop_timer = nh_.createTimer(ros::Duration((double)1/update_rate), &ObstacleFeed::updateObstacles, this);
            update_service = nh_.advertiseService("update_trigger", &ObstacleFeed::UpdateCallback, this);
            update_service_int = nh_.advertiseService("update_trigger_int", &ObstacleFeed::UpdateCallbackInt, this);

            // Check predefined obstacles for errors
            if (!(lmpcc_obstacle_feed_config_->obst_pose_x_.size() == lmpcc_obstacle_feed_config_->obst_pose_y_.size() && lmpcc_obstacle_feed_config_->obst_pose_x_.size() == lmpcc_obstacle_feed_config_->obst_pose_heading_.size() && lmpcc_obstacle_feed_config_->obst_pose_x_.size() == lmpcc_obstacle_feed_config_->obst_dim_minor_.size() && lmpcc_obstacle_feed_config_->obst_pose_x_.size() == lmpcc_obstacle_feed_config_->obst_dim_major_.size()))
            {
                ROS_ERROR("Predefined obstacle arrays are not of the same length!");

                return false;
            }


        }
        else if (lmpcc_obstacle_feed_config_->obstacle_feed_mode_ == 1)
        {
            ROS_WARN("In this mode, tracked obstacles by the OptiTrack system are forwarded");

            int update_rate = lmpcc_obstacle_feed_config_->update_rate_;

            optitrack_sub = nh_.subscribe(lmpcc_obstacle_feed_config_->sub_optitrack_, 1, &ObstacleFeed::optitrackCallback, this);
        }
        else if (lmpcc_obstacle_feed_config_->obstacle_feed_mode_ == 0)
        {
            ROS_WARN("In this mode, clustered pointcloud obstacles are forwarded");

            obstacles_sub = nh_.subscribe(lmpcc_obstacle_feed_config_->sub_pedestrians_, 1, &ObstacleFeed::pedestriansCallback, this);
        }
        else if(lmpcc_obstacle_feed_config_->obstacle_feed_mode_ == 4)
        {
            ROS_WARN("In this mode, data from Pedsim is forwarded");
            state_sub = nh_.subscribe(lmpcc_obstacle_feed_config_->sub_pedsim_,1, &ObstacleFeed::PedsimCallback, this);
            link_state_client_ = nh_.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");
        }
        else{
            ROS_ERROR("UNDEFINED MODE");
        }
        ROS_INFO_STREAM("ObstacleFeed Initialized");
        return true;
    }
    else
    {
        return false;
    }
}

void ObstacleFeed::spinNode()
{
  ros::spin();
}

void ObstacleFeed::clearDataMember()
{
    ;
}

void ObstacleFeed::reconfigureCallback(lmpcc_obstacle_feed::ObstacleFeedConfig& config, uint32_t level)
{
    minV_ = config.minV;
    maxV_ = config.maxV;
    //N_obstacles_ = config.N_obstacles;
    //distance_ = config.distance_threshold;
    obstacle_size_ = config.obstacle_size;
}

bool ObstacleFeed::UpdateCallbackInt(lmpcc_msgs::IntTrigger::Request& request, lmpcc_msgs::IntTrigger::Response& response)
{
    int rate = request.value;

    for (int obst_it = 0; obst_it < obstacles_.lmpcc_obstacles.size(); obst_it++)
    {
        obstacles_.lmpcc_obstacles[obst_it].trajectory.header.stamp = ros::Time::now();

        obstacles_.lmpcc_obstacles[obst_it].pose.position.x = obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[0].pose.position.x + ((double) 1/rate )*lmpcc_obstacle_feed_config_->v_x_.at(obst_it);
        obstacles_.lmpcc_obstacles[obst_it].pose.position.y = obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[0].pose.position.y + ((double) 1/rate )*lmpcc_obstacle_feed_config_->v_y_.at(obst_it);

        for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++)
        {
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.stamp = ros::Time::now();
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;

            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.stamp = ros::Time::now();
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.x = obstacles_.lmpcc_obstacles[obst_it].pose.position.x + dt_*traj_it*lmpcc_obstacle_feed_config_->v_x_.at(obst_it);
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.y = obstacles_.lmpcc_obstacles[obst_it].pose.position.y + dt_*traj_it*lmpcc_obstacle_feed_config_->v_y_.at(obst_it);
        }
    }

    return true;
}

bool ObstacleFeed::UpdateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{

    for (int obst_it = 0; obst_it < obstacles_.lmpcc_obstacles.size(); obst_it++)
    {
        obstacles_.lmpcc_obstacles[obst_it].trajectory.header.stamp = ros::Time::now();

        obstacles_.lmpcc_obstacles[obst_it].pose = obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[1].pose;

        for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++)
        {
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.stamp = ros::Time::now();
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;

            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.stamp = ros::Time::now();
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.x = obstacles_.lmpcc_obstacles[obst_it].pose.position.x + dt_*traj_it*lmpcc_obstacle_feed_config_->v_x_.at(obst_it);
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.y = obstacles_.lmpcc_obstacles[obst_it].pose.position.y + dt_*traj_it*lmpcc_obstacle_feed_config_->v_y_.at(obst_it);
        }
    }

    return true;
}

bool ObstacleFeed::UpdateCallback()
{

    for (int obst_it = 0; obst_it < obstacles_.lmpcc_obstacles.size(); obst_it++)
    {
        obstacles_.lmpcc_obstacles[obst_it].trajectory.header.stamp = ros::Time::now();

        obstacles_.lmpcc_obstacles[obst_it].pose = obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[1].pose;

        for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++)
        {
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.stamp = ros::Time::now();
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;

            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.stamp = ros::Time::now();
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.x = obstacles_.lmpcc_obstacles[obst_it].pose.position.x + dt_*traj_it*lmpcc_obstacle_feed_config_->v_x_.at(obst_it);
            obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.y = obstacles_.lmpcc_obstacles[obst_it].pose.position.y + dt_*traj_it*lmpcc_obstacle_feed_config_->v_y_.at(obst_it);
        }
    }

    return true;
}

void ObstacleFeed::updateObstacles(const ros::TimerEvent& event)
{
    if (lmpcc_obstacle_feed_config_->obstacle_feed_mode_ == 2)
    {
        UpdateCallback();
    }
    // Publish ellipsoid obstacles
    publishObstacles(obstacles_);
    // Visualize ellipsoid obstacles
    visualizeObstacles(obstacles_);
}


void ObstacleFeed::optitrackCallback(const nav_msgs::Path& predicted_path)
{

//    if (predicted_path.header.frame_id == "1"){
//
//        for (int path_it = 0; path_it < ACADO_N ; path_it++)
//        {
//            obst1_x[path_it] = predicted_path.poses[path_it].pose.position.x + 1.55;
//            obst1_y[path_it] = predicted_path.poses[path_it].pose.position.y + 2.75;
//        }
//    }
//    else if  (predicted_path.header.frame_id == "2"){
//
//        for (int path_it = 0; path_it < ACADO_N ; path_it++)
//        {
//            obst2_x[path_it] = predicted_path.poses[path_it].pose.position.x + 1.55;
//            obst2_y[path_it] = predicted_path.poses[path_it].pose.position.y + 2.75;
//        }
//
//    }else if  (predicted_path.header.frame_id == "3"){
//
//    }
//    else {
//        ROS_INFO_STREAM("Obstacle id not recognized");
//    }
}

void ObstacleFeed::detectionsCallback(const vision_msgs::Detection3DArray& objects)
{
    //  ROS_INFO_STREAM("Obstacles callback!");

    objectArray_ = objects;

    std::vector<uint32_t> objectIDs;
    std::vector<double> objectDistances;
    double distance, volume;

    vision_msgs::Detection3DArray local_objects;
    lmpcc_msgs::lmpcc_obstacle_array ellipses;
    lmpcc_msgs::lmpcc_obstacle_array local_ellipses;
    lmpcc_msgs::lmpcc_obstacle ellipse;

    for (int object_it = 0; object_it < objectArray_.detections.size(); object_it++)
    {
//        // SHIFTING ALL OBSTACLES IN SPACE
//        objectArray_.objects[object_it].pose.position.y = objectArray_.objects[object_it].pose.position.y + 2;

        // Compute distance of obstacle to robot
        distance = sqrt(pow(objectArray_.detections[object_it].bbox.center.position.x,2) + pow(objectArray_.detections[object_it].bbox.center.position.y,2));
        volume = objectArray_.detections[object_it].bbox.size.x*objectArray_.detections[object_it].bbox.size.y*objectArray_.detections[object_it].bbox.size.z;

        // If distance is smaller than defined bound, add to obstacles
        if (distance < distance_ && volume > minV_ && volume < maxV_){
            local_objects.detections.push_back(objectArray_.detections[object_it]);
            objectDistances.push_back(distance);
        }
    }

    // For all obstacles, fit an ellipse
    for (int local_obst_it = 0; local_obst_it < local_objects.detections.size(); local_obst_it++)
    {
        ellipse = FitEllipse(local_objects.detections[local_obst_it],objectDistances[local_obst_it]);
        ellipses.lmpcc_obstacles.push_back(ellipse);

    }

    // Order obstacles according to distance
    OrderObstacles(ellipses);

    local_ellipses.lmpcc_obstacles.clear();

    // Transform and add to local obstacles upto a defined bound
    for (int ellipses_it = 0; ellipses_it < N_obstacles_ && ellipses_it < ellipses.lmpcc_obstacles.size(); ellipses_it++)
    {
        ZRotToQuat(ellipses.lmpcc_obstacles[ellipses_it].pose);
        transformPose(lmpcc_obstacle_feed_config_->robot_frame_,lmpcc_obstacle_feed_config_->planning_frame_,ellipses.lmpcc_obstacles[ellipses_it].pose);
        QuatToZRot(ellipses.lmpcc_obstacles[ellipses_it].pose);
        local_ellipses.lmpcc_obstacles.push_back(ellipses.lmpcc_obstacles[ellipses_it]);
    }

    // Publish and visualize obstacles
    publishObstacles(local_ellipses);
    visualizeObstacles(local_ellipses);

}

void ObstacleFeed::PedsimCallback(const pedsim_msgs::TrackedPersons& person)
{
    //ROS_INFO_STREAM("PEDSIM callback!");

    double Xp, Yp;
    double q1, q2, q3, q4;
    double mag_q, distance;
    float major_semiaxis, minor_semiaxis, z_rotation;
    std::vector<uint32_t> objectIds;
    std::vector<double> objectDistances;
    lmpcc_msgs::lmpcc_obstacle_array filter_obstacles;
    lmpcc_msgs::lmpcc_obstacle obst;
    lmpcc_msgs::lmpcc_obstacle obst1;
    lmpcc_msgs::lmpcc_obstacle_array local_obstacles;
    lmpcc_msgs::lmpcc_obstacle_array ellipses;
    lmpcc_msgs::lmpcc_obstacle_array local_ellipses;
    lmpcc_msgs::lmpcc_obstacle ellipse;
    double f = lmpcc_obstacle_feed_config_->prediction_horizon_/lmpcc_obstacle_feed_config_->discretization_steps_;
    geometry_msgs::PoseStamped test;

    //resize the vectors
    ellipse.trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst.trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst.major_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst.minor_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst1.trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst1.major_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst1.minor_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    // hard code to link gazebo peds with pedsim
    /*test.pose = person.tracks[0].pose.pose;
    test.header.frame_id = "ped_link_"+std::to_string(1);
    ped_pub_.publish(test);
    test.pose = person.tracks[1].pose.pose;
    test.header.frame_id = "ped_link_"+std::to_string(2);
    ped_pub2_.publish(test);
    test.pose = person.tracks[2].pose.pose;
    test.header.frame_id = "ped_link_"+std::to_string(3);
    ped_pub3_.publish(test);
    test.pose = person.tracks[3].pose.pose;
    test.header.frame_id = "ped_link_"+std::to_string(4);
    ped_pub4_.publish(test);
     */
    /*gazebo_msgs::SetLinkState link;
    link.request.link_state.link_name="ped_link_1";
    link.request.link_state.pose = person.tracks[0].pose.pose;
    //link.request.link_state.reference_frame = "odom";
    link_state_client_.call(link);

    link.request.link_state.link_name="ped_link_2";
    link.request.link_state.pose = person.tracks[1].pose.pose;
    link_state_client_.call(link);
    */
    //link_state_pub_.publish(link);
    // hard code to link gazebo peds with pedsim
    for(int person_it=0;person_it<person.tracks.size();person_it++) { // iterates over obstacles

        obst.pose=person.tracks[person_it].pose.pose;
        obst.id = person.tracks[person_it].track_id;
        obst.velocity=person.tracks[person_it].twist.twist;
        test.pose = person.tracks[person_it].pose.pose;
        test.header.frame_id = "ped_link_"+std::to_string(person_it+1);
        link_pubs_[person_it]->publish(test);

        people_msg_.header.stamp=ros::Time::now();
        people_msg_.people[person_it].name="ped_link_"+std::to_string(person_it+1);
        people_msg_.people[person_it].position.x=person.tracks[person_it].pose.pose.position.x;
        people_msg_.people[person_it].position.y=person.tracks[person_it].pose.pose.position.y;
        people_msg_.people[person_it].position.z=0;
        people_msg_.people[person_it].velocity.x=person.tracks[person_it].twist.twist.linear.x;
        people_msg_.people[person_it].velocity.y=person.tracks[person_it].twist.twist.linear.y;
        people_msg_.people[person_it].reliability=1.0;
        people_pub.publish(people_msg_);
        //ensure magnitude of quaternion is one
        q1 = obst.pose.orientation.x;
        q2 = obst.pose.orientation.y;
        q3 = obst.pose.orientation.z;
        q4 = obst.pose.orientation.w;
        mag_q = sqrt((q1 * q1) + (q2 * q2) + (q3 * q3) + (q4 * q4));

        if (mag_q != 1) {

            //ROS_WARN("Quaternion magnitude not equal to one, making required modifications");
            obst.pose.orientation.x = 0;
            obst.pose.orientation.y = 0;
            obst.pose.orientation.z = 0;
            obst.pose.orientation.w = 1;

        }

        //getting the major and minor semi axis values
        obst.minor_semiaxis[0]= lmpcc_obstacle_feed_config_->obst_dim_major_[0];
        obst.major_semiaxis[0]= lmpcc_obstacle_feed_config_->obst_dim_minor_[0];
        obst.trajectory.poses[0].pose = obst.pose;
        if(!lmpcc_obstacle_feed_config_->kalman_){
            for(int i = 1; i < lmpcc_obstacle_feed_config_->discretization_steps_; i++){
                obst.trajectory.poses[i].pose.position.x = obst.trajectory.poses[i-1].pose.position.x+person.tracks[person_it].twist.twist.linear.x*f;
                obst.trajectory.poses[i].pose.position.y = obst.trajectory.poses[i-1].pose.position.y+person.tracks[person_it].twist.twist.linear.y*f;
                obst.trajectory.poses[i].pose.orientation = obst.trajectory.poses[i-1].pose.orientation;
                obst.minor_semiaxis[i] = obst.minor_semiaxis[0];
                obst.major_semiaxis[i] = obst.major_semiaxis[0];
            }
        }
        else{
            obst.trajectory = filters_[person_it]->updateFilter(obst.trajectory.poses[0].pose);
        }

        local_obstacles.lmpcc_obstacles.push_back(obst);
    }


    for(int i = 0; i< local_obstacles.lmpcc_obstacles.size(); i++) {

        // transform the pose to base_link in order to calculate the distance to the obstacle
        transformPose(person.header.frame_id, "base_link", local_obstacles.lmpcc_obstacles[i].pose);
        //transformTwist(person.header.frame_id, "base_link", local_obstacles.lmpcc_obstacles[i].velocity);
        //get obstacle coordinates in base_link frame
        Xp = local_obstacles.lmpcc_obstacles[i].pose.position.x;
        Yp = local_obstacles.lmpcc_obstacles[i].pose.position.y;

        //distance between the Prius and the obstacle
        distance = sqrt(pow(Xp, 2) + pow(Yp, 2));

        //ROS_WARN_STREAM("distance to obstacle: " << distance);

        //transform the pose back to planning_frame for further calculations
        transformPose("base_link", lmpcc_obstacle_feed_config_->planning_frame_,
                      local_obstacles.lmpcc_obstacles[i].pose);


        //get the bounding box pose measurements for the obstacle
        obst1.id = local_obstacles.lmpcc_obstacles[i].id;
        obst1.pose = local_obstacles.lmpcc_obstacles[i].pose;
        obst1.velocity = local_obstacles.lmpcc_obstacles[i].velocity;
        //get the major and minor semi axis readings of the stored obstacles
        for(int j=0; j< obst1.major_semiaxis.size(); j++){

            obst1.major_semiaxis[j] = local_obstacles.lmpcc_obstacles[i].major_semiaxis[j];
            obst1.minor_semiaxis[j] = local_obstacles.lmpcc_obstacles[i].minor_semiaxis[j];

        }

        //get the trajectory pose readings of the stored obstacles
        for(int k=0; k< lmpcc_obstacle_feed_config_->discretization_steps_; k++){

            obst1.trajectory.poses[k].pose = local_obstacles.lmpcc_obstacles[i].trajectory.poses[k].pose;
            //obst1.trajectory.poses[k].pose.position.y = local_obstacles.lmpcc_obstacles[i].trajectory.poses[k].pose.position.y;
            //transform the pose back to planning_frame for further calculations
            if(person.header.frame_id!=lmpcc_obstacle_feed_config_->planning_frame_)
                transformPose(person.header.frame_id, lmpcc_obstacle_feed_config_->planning_frame_,obst1.trajectory.poses[k].pose);

        }

        //filter out the obstacles that are farther away than the threshold distance value
        if (distance < distance_) {

            filter_obstacles.lmpcc_obstacles.push_back(obst1);
            objectDistances.push_back(distance);

        }

    }

    //fit the ellipse to the obstacles
    for(int filt_obst_it = 0; filt_obst_it < filter_obstacles.lmpcc_obstacles.size(); filt_obst_it ++) {

        //fitting the ellipse
        ellipse = FitEllipse(filter_obstacles.lmpcc_obstacles[filt_obst_it], objectDistances[filt_obst_it]);
        ellipse.id = filter_obstacles.lmpcc_obstacles[filt_obst_it].id;
        //estimating the ellipse trajectory over the defined horizon
        for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++) {
            ellipse.trajectory.poses[traj_it].header.stamp = ros::Time::now();
            ellipse.trajectory.poses[traj_it].header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;
            ellipse.trajectory.header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;
            ellipse.trajectory.poses[traj_it].pose = filter_obstacles.lmpcc_obstacles[filt_obst_it].trajectory.poses[traj_it].pose;
        }

        //store the calculated ellipse
        ellipses.lmpcc_obstacles.push_back(ellipse);

    }

    //order the stored ellipses with the closest one being first
    OrderObstacles(ellipses);

    //find the minimum out of the number of detected obstacles and the default number of eligible obstacles to implement the algorithm
    int n = std::min(int(ellipses.lmpcc_obstacles.size()),N_obstacles_);
    //ROS_INFO_STREAM("Publish and visualize obstacles: " << n);
    // store the ordered ellipses in a vector local_ellipses
    for (int ellipses_it = 0; ellipses_it < n; ellipses_it++) {

        local_ellipses.lmpcc_obstacles.push_back(ellipses.lmpcc_obstacles[ellipses_it]);
    }

    //if the number of detected obstacle is less than the default number, randomly initialize the remaining number of required obstacles
    for (int ellipses_it = n; ellipses_it < N_obstacles_ ; ellipses_it++)
    {
        ellipse.pose.position.x = DEFAULT_OBSTACLE_SIZE;
        ellipse.pose.position.y = DEFAULT_OBSTACLE_SIZE;
        ellipse.id=0;
        ellipse.minor_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
        ellipse.major_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
        for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++)
        {
            ellipse.trajectory.poses[traj_it].header.stamp = ros::Time::now();
            ellipse.trajectory.poses[traj_it].header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;
            ellipse.trajectory.header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;
            ellipse.trajectory.poses[traj_it].pose.position.x = DEFAULT_OBSTACLE_SIZE;
            ellipse.trajectory.poses[traj_it].pose.position.y = DEFAULT_OBSTACLE_SIZE;
        }

        local_ellipses.lmpcc_obstacles.push_back(ellipse);
    }

    //publish and visualize the detected obstacles

    if(local_ellipses.lmpcc_obstacles.size()>0){

        publishObstacles(local_ellipses);
        visualizeObstacles(local_ellipses);

    }
}

void ObstacleFeed::pedestriansCallback(const spencer_tracking_msgs::TrackedPersons& person)
{
    //ROS_INFO_STREAM("Pedestrian callback!");F

    double Xp, Yp;
    double q1, q2, q3, q4;
    double mag_q, distance;
    float major_semiaxis, minor_semiaxis, z_rotation;
    std::vector<uint32_t> objectIds;
    std::vector<double> objectDistances;
    lmpcc_msgs::lmpcc_obstacle_array filter_obstacles;
    lmpcc_msgs::lmpcc_obstacle obst;
    lmpcc_msgs::lmpcc_obstacle obst1;
    lmpcc_msgs::lmpcc_obstacle_array local_obstacles;
    lmpcc_msgs::lmpcc_obstacle_array ellipses;
    lmpcc_msgs::lmpcc_obstacle_array local_ellipses;
    lmpcc_msgs::lmpcc_obstacle ellipse;
    double f = lmpcc_obstacle_feed_config_->prediction_horizon_/lmpcc_obstacle_feed_config_->discretization_steps_;

    //resize the vectors
    ellipse.trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst.trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst.major_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst.minor_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst1.trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst1.major_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    obst1.minor_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);

    for(int person_it=0;person_it<person.tracks.size();person_it++) { // iterates over obstacles

        obst.pose=person.tracks[person_it].pose.pose;

        //ensure magnitude of quaternion is one
        q1 = obst.pose.orientation.x;
        q2 = obst.pose.orientation.y;
        q3 = obst.pose.orientation.z;
        q4 = obst.pose.orientation.w;
        mag_q = sqrt((q1 * q1) + (q2 * q2) + (q3 * q3) + (q4 * q4));

        if (mag_q != 1) {

            //ROS_WARN("Quaternion magnitude not equal to one, making required modifications");
            obst.pose.orientation.x = 0;
            obst.pose.orientation.y = 0;
            obst.pose.orientation.z = 0;
            obst.pose.orientation.w = 1;

        }

        //getting the major and minor semi axis values
        obst.minor_semiaxis[0]= lmpcc_obstacle_feed_config_->obst_dim_major_[0];
        obst.major_semiaxis[0]= lmpcc_obstacle_feed_config_->obst_dim_minor_[0];
        obst.trajectory.poses[0].pose = obst.pose;
        if(!lmpcc_obstacle_feed_config_->kalman_){
            for(int i = 1; i < lmpcc_obstacle_feed_config_->discretization_steps_; i++){
                obst.trajectory.poses[i].pose.position.x = obst.trajectory.poses[i-1].pose.position.x+person.tracks[person_it].twist.twist.linear.x*f;
                obst.trajectory.poses[i].pose.position.y = obst.trajectory.poses[i-1].pose.position.y+person.tracks[person_it].twist.twist.linear.y*f;
                obst.trajectory.poses[i].pose.orientation = obst.trajectory.poses[i-1].pose.orientation;
                obst.minor_semiaxis[i] = obst.minor_semiaxis[0];
                obst.major_semiaxis[i] = obst.major_semiaxis[0];
            }
        }
        else{
            obst.trajectory = filters_[person_it]->updateFilter(obst.trajectory.poses[0].pose);
        }

        local_obstacles.lmpcc_obstacles.push_back(obst);
    }


    for(int i = 0; i< local_obstacles.lmpcc_obstacles.size(); i++) {

        // transform the pose to base_link in order to calculate the distance to the obstacle
        transformPose(person.header.frame_id, "base_link", local_obstacles.lmpcc_obstacles[i].pose);

        //get obstacle coordinates in base_link frame
        Xp = local_obstacles.lmpcc_obstacles[i].pose.position.x;
        Yp = local_obstacles.lmpcc_obstacles[i].pose.position.y;

        //distance between the Prius and the obstacle
        distance = sqrt(pow(Xp, 2) + pow(Yp, 2));

        //ROS_WARN_STREAM("distance to obstacle: " << distance);

        //transform the pose back to planning_frame for further calculations
        transformPose("base_link", lmpcc_obstacle_feed_config_->planning_frame_,
                      local_obstacles.lmpcc_obstacles[i].pose);


        //get the bounding box pose measurements for the obstacle
        obst1.pose = local_obstacles.lmpcc_obstacles[i].pose;
        //get the major and minor semi axis readings of the stored obstacles
        for(int j=0; j< obst1.major_semiaxis.size(); j++){

            obst1.major_semiaxis[j] = local_obstacles.lmpcc_obstacles[i].major_semiaxis[j];
            obst1.minor_semiaxis[j] = local_obstacles.lmpcc_obstacles[i].minor_semiaxis[j];

        }

        //get the trajectory pose readings of the stored obstacles
        for(int k=0; k< lmpcc_obstacle_feed_config_->discretization_steps_; k++){

            obst1.trajectory.poses[k].pose = local_obstacles.lmpcc_obstacles[i].trajectory.poses[k].pose;
            //obst1.trajectory.poses[k].pose.position.y = local_obstacles.lmpcc_obstacles[i].trajectory.poses[k].pose.position.y;
            //transform the pose back to planning_frame for further calculations
            if(person.header.frame_id!=lmpcc_obstacle_feed_config_->planning_frame_)
                transformPose(person.header.frame_id, lmpcc_obstacle_feed_config_->planning_frame_,obst1.trajectory.poses[k].pose);

        }

        //filter out the obstacles that are farther away than the threshold distance value
        if (distance < distance_) {

            filter_obstacles.lmpcc_obstacles.push_back(obst1);
            objectDistances.push_back(distance);

        }

    }

    //fit the ellipse to the obstacles
    for(int filt_obst_it = 0; filt_obst_it < filter_obstacles.lmpcc_obstacles.size(); filt_obst_it ++) {

        //fitting the ellipse
        ellipse = FitEllipse(filter_obstacles.lmpcc_obstacles[filt_obst_it], objectDistances[filt_obst_it]);

        //estimating the ellipse trajectory over the defined horizon
        for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++) {
            ellipse.trajectory.poses[traj_it].header.stamp = ros::Time::now();
            ellipse.trajectory.poses[traj_it].header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;
            ellipse.trajectory.header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;
            ellipse.trajectory.poses[traj_it].pose = filter_obstacles.lmpcc_obstacles[filt_obst_it].trajectory.poses[traj_it].pose;
        }

        //store the calculated ellipse
        ellipses.lmpcc_obstacles.push_back(ellipse);

    }

    //order the stored ellipses with the closest one being first
    OrderObstacles(ellipses);

    //find the minimum out of the number of detected obstacles and the default number of eligible obstacles to implement the algorithm
    int n = std::min(int(ellipses.lmpcc_obstacles.size()),N_obstacles_);
    //ROS_INFO_STREAM("Publish and visualize obstacles: " << n);
    // store the ordered ellipses in a vector local_ellipses
    for (int ellipses_it = 0; ellipses_it < n; ellipses_it++) {

        local_ellipses.lmpcc_obstacles.push_back(ellipses.lmpcc_obstacles[ellipses_it]);
    }

    //if the number of detected obstacle is less than the default number, randomly initialize the remaining number of required obstacles
    for (int ellipses_it = n; ellipses_it < N_obstacles_ ; ellipses_it++)
    {
        ellipse.pose.position.x = 1000;
        ellipse.pose.position.y = 1000;
        ellipse.minor_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
        ellipse.major_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
        for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++)
        {
            ellipse.trajectory.poses[traj_it].header.stamp = ros::Time::now();
            ellipse.trajectory.poses[traj_it].header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;
            ellipse.trajectory.header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;
            ellipse.trajectory.poses[traj_it].pose.position.x = 1000;
            ellipse.trajectory.poses[traj_it].pose.position.y = 1000;
        }

        local_ellipses.lmpcc_obstacles.push_back(ellipse);

    }


    //publish and visualize the detected obstacles

    if(local_ellipses.lmpcc_obstacles.size()>0){

        publishObstacles(local_ellipses);
        visualizeObstacles(local_ellipses);

    }
}

void ObstacleFeed::publishObstacles(const lmpcc_msgs::lmpcc_obstacle_array& obstacles)
{
    //ROS_INFO_STREAM("publishObstacles");
    obstacles_pub.publish(obstacles);
}

void ObstacleFeed::visualizeObstacles(const lmpcc_msgs::lmpcc_obstacle_array& obstacles) {

    // Initialize markers for visualization
    visualization_msgs::Marker marker;
    visualization_msgs::MarkerArray markerArray;

    // Loop over obstacles in obstacle array
    for (int obst_it = 0; obst_it < obstacles.lmpcc_obstacles.size(); obst_it++) {
        marker.header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;           // Add frame of obstacle
        marker.header.stamp = ros::Time::now();                 // Add timestamp
        marker.id = obst_it*100;                                    // Obstacle ID
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.pose = obstacles.lmpcc_obstacles[obst_it].pose;
        ZRotToQuat(marker.pose);
        // Get Quaternion rotation
        marker.scale.x = 2*obstacles.lmpcc_obstacles[obst_it].major_semiaxis[0];     // Marker is specified by diameter, not radius!
        marker.scale.y = 2*obstacles.lmpcc_obstacles[obst_it].minor_semiaxis[0];
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerArray.markers.push_back(marker);
        marker.color.a = 0.2;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        for (int traj_it = 0; traj_it < obstacles.lmpcc_obstacles[obst_it].trajectory.poses.size(); traj_it++) {
            marker.id = obst_it * 100 + traj_it;                                    // Obstacle ID
            marker.pose = obstacles.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose;
	    marker.scale.x = 2*obstacles.lmpcc_obstacles[obst_it].major_semiaxis[traj_it];     // Marker is specified by diameter, not radius!
            marker.scale.y = 2*obstacles.lmpcc_obstacles[obst_it].minor_semiaxis[traj_it];
            markerArray.markers.push_back(marker);
        }
    }

    visualize_obstacles_pub.publish(markerArray);

    if (obstacles.lmpcc_obstacles.size() == 1) {
        obst1_path_pub.publish(obstacles.lmpcc_obstacles[0].trajectory);
    }
    if (obstacles.lmpcc_obstacles.size() == 2) {
        obst1_path_pub.publish(obstacles.lmpcc_obstacles[0].trajectory);
        obst2_path_pub.publish(obstacles.lmpcc_obstacles[1].trajectory);
    }
}

bool CompareObstacleDistance(lmpcc_msgs::lmpcc_obstacle const &obst1, lmpcc_msgs::lmpcc_obstacle const &obst2) { return (obst1.distance < obst2.distance); }

void ObstacleFeed::OrderObstacles(lmpcc_msgs::lmpcc_obstacle_array& ellipses)
{
    // Create vector of obstacles
    if(ellipses.lmpcc_obstacles.size()>0){
        std::vector<lmpcc_msgs::lmpcc_obstacle> ellipsesVector;
        ellipsesVector = ellipses.lmpcc_obstacles;

        // Sort vector according to distances
        std::sort(ellipsesVector.begin(),ellipsesVector.end(), CompareObstacleDistance);

        // Write vector of sorted obstacles to obstacles structure
        ellipses.lmpcc_obstacles = ellipsesVector;
    }
//    // print out content:
//    std::cout << "myvector contains:";
//    for (int it = 0; it < ellipses.lmpcc_obstacles.size(); it++)
//        std::cout << ' ' << ellipses.lmpcc_obstacles[it].distance;
//    std::cout << '\n';
}

lmpcc_msgs::lmpcc_obstacle ObstacleFeed::FitEllipse(const lmpcc_msgs::lmpcc_obstacle& object, const double& distance)
{
    //ROS_INFO_STREAM("FitEllipse");
    lmpcc_msgs::lmpcc_obstacle ellipse;
    ellipse.trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    ellipse.major_semiaxis.resize(object.major_semiaxis.size());
    ellipse.minor_semiaxis.resize(object.minor_semiaxis.size());
    for(int k=0; k< object.major_semiaxis.size(); k++){
        ellipse.major_semiaxis[k] = object.major_semiaxis[k];
        ellipse.minor_semiaxis[k] = object.minor_semiaxis[k];
    }

    ellipse.distance = distance;
    ellipse.pose = object.pose;
    ellipse.velocity = object.velocity;
    return ellipse;
}

lmpcc_msgs::lmpcc_obstacle ObstacleFeed::FitEllipse(const vision_msgs::Detection3D& object, const double& distance)
{
    //ROS_INFO_STREAM("FitEllipse");
    lmpcc_msgs::lmpcc_obstacle ellipse;
    ellipse.trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    ellipse.major_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    ellipse.minor_semiaxis.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    ellipse.major_semiaxis[0] = obstacle_size_; // sqrt(pow(object.dimensions.x,2) + pow(object.dimensions.y,2))/2;
    ellipse.minor_semiaxis[0] = obstacle_size_; // sqrt(pow(object.dimensions.x,2) + pow(object.dimensions.y,2))/2;

    ellipse.distance = distance;
    ellipse.pose = object.bbox.center;
    return ellipse;
}

void ObstacleFeed::QuatToZRot(geometry_msgs::Pose& pose)
{
    double ysqr = pose.orientation.y * pose.orientation.y;
    double t3 = +2.0 * (pose.orientation.w * pose.orientation.z + pose.orientation.x * pose.orientation.y);
    double t4 = +1.0 - 2.0 * (ysqr + pose.orientation.z * pose.orientation.z);

    pose.orientation.z = atan2(t3, t4);
}

void ObstacleFeed::ZRotToQuat(geometry_msgs::Pose& pose)
{
    pose.orientation.w = cos(pose.orientation.z * 0.5);
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = sin(pose.orientation.z * 0.5);
}

bool ObstacleFeed::transformTwist(const std::string& from, const std::string& to, geometry_msgs::Twist& twist)
{
    bool transform = false;
    tf::StampedTransform stamped_tf;

    geometry_msgs::PoseStamped stampedPose_in, stampedPose_out;

    stampedPose_in.pose.position.x = twist.linear.x;
    stampedPose_in.pose.position.y = twist.linear.y;
//    stampedPose_in.header.stamp = ros::Time::now();
    stampedPose_in.header.frame_id = from;

    // make sure source and target frame exist
    if (tf_listener_.frameExists(to) && tf_listener_.frameExists(from))
    {
        try
        {
            // find transforamtion between souce and target frame
            tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.01));
            tf_listener_.transformPose(to, stampedPose_in, stampedPose_out);

            transform = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("MPCC::getTransform: %s", ex.what());
        }
    }

    else
    {
        ROS_WARN("MPCC::getTransform: '%s' or '%s' frame doesn't exist, pass existing frame",from.c_str(), to.c_str());
    }

    twist.linear.x = stampedPose_out.pose.position.x;
    twist.linear.y = stampedPose_out.pose.position.y;

    return transform;
}

bool ObstacleFeed::transformPose(const std::string& from, const std::string& to, geometry_msgs::Pose& pose)
{
    bool transform = false;
    tf::StampedTransform stamped_tf;

    geometry_msgs::PoseStamped stampedPose_in, stampedPose_out;

    stampedPose_in.pose = pose;
//    stampedPose_in.header.stamp = ros::Time::now();
    stampedPose_in.header.frame_id = from;

    // make sure source and target frame exist
    if (tf_listener_.frameExists(to) && tf_listener_.frameExists(from))
    {
        try
        {
            // find transforamtion between souce and target frame
            tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.02));
            tf_listener_.transformPose(to, stampedPose_in, stampedPose_out);

            transform = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("MPCC::getTransform: %s", ex.what());
        }
    }

    else
    {
        ROS_WARN("MPCC::getTransform: '%s' or '%s' frame doesn't exist, pass existing frame",from.c_str(), to.c_str());
    }

    pose = stampedPose_out.pose;

    return transform;
}
