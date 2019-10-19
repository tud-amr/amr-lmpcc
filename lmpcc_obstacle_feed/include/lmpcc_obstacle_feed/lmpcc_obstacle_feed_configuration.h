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

#include <string>

using namespace std;

class lmpcc_obstacle_feed_configuration
{

public:

    lmpcc_obstacle_feed_configuration();

    ~lmpcc_obstacle_feed_configuration();

    bool initialize();
    bool updateConfiguration();

    bool initialize_success_;

    int obstacle_feed_mode_;

    bool activate_debug_output_;
    bool activate_visualization_;

    int update_rate_;
    int discretization_steps_;
    double prediction_horizon_;

    string planning_frame_;
    string robot_frame_;

    string pub_obstacles_;
    string pub_obstacles_vis_;

    string sub_detections_;
    string sub_optitrack_;
    string sub_pedestrians_;
    string sub_prius_;
    string sub_pedsim_;
    string pub_people_;
    int test_;

    bool kalman_;

    double distance_threshold_;
    int obstacle_threshold_;
    int obstacle_number_;
    double min_obstacle_volume_;
    double max_obstacle_volume_;
    double obstacle_size_;

    std::vector<double> obst_pose_x_;
    std::vector<double> obst_pose_y_;
    std::vector<double> obst_pose_heading_;
    std::vector<double> v_x_;
    std::vector<double> v_y_;
    std::vector<double> obst_dim_minor_;
    std::vector<double> obst_dim_major_;

private:

    void free_allocated_memory();

    void print_configuration_parameter();

};
