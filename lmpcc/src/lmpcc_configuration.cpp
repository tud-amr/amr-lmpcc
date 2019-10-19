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
 * Authors: Bruno Brito email: bruno.debrito@tudelft.nl
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
#include <lmpcc/lmpcc_configuration.h>

LMPCC_configuration::LMPCC_configuration()
{
    initialize_success_ = false;
}

LMPCC_configuration::~LMPCC_configuration()
{
    free_allocated_memory();
}

// read configuration parameter from parameter server
bool LMPCC_configuration::initialize()
{
    // Initialize nodehandles
    ros::NodeHandle nh_config;
    ros::NodeHandle nh;

    /** Simulation mode **/

    if (!nh.getParam ("simulation_mode", simulation_mode_) )
    {
        ROS_WARN(" Parameter 'simulation_mode' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    if (!nh.getParam ("gazebo_simulation", gazebo_simulation_) )
    {
        ROS_WARN(" Parameter 'gazebo_simulation' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    /** Debug modes **/
    nh.param("activate_output", activate_output_, bool(true));
    nh.param("activate_debug_output", activate_debug_output_, bool(false));
    nh.param("activate_timing_output", activate_timing_output_, bool(false));
    nh.param("activate_visualization", activate_visualization_, bool(false));
    nh.param("activate_feedback_message", activate_feedback_message_, bool(false));

    /** controller frequency in Hz **/
    nh.param("controller_frequency", controller_frequency_, double(20.0)); // 50 hz

    /** dimensions of controlled vehicle **/
    // Number of ego-vehicle representing discs
    if (!nh.getParam ("robot_dimensions/n_discs", n_discs_) )
    {
        ROS_WARN(" Parameter 'n_discs' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // Length of the controlled platform
    if (!nh.getParam ("robot_dimensions/ego_l", ego_l_) )
    {
        ROS_WARN(" Parameter 'ego_l' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // Width of the controlled platform
    if (!nh.getParam ("robot_dimensions/ego_w", ego_w_) )
    {
        ROS_WARN(" Parameter 'ego_w' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    /** OCP dimensions **/
    // OCP state dimension
    if (!nh.getParam("state_dim", state_dim_) )
    {
        ROS_WARN(" Parameter 'state_dim' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // OCP control dimension
    if (!nh.getParam ("control_dim", control_dim_) )
    {
        ROS_WARN(" Parameter 'control_dim' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    /** publish and subscribe topic definitions **/
    // Control command publication topic
    if (!nh.getParam ("publish/cmd", cmd_) )
    {
        ROS_WARN(" Parameter 'output_cmd' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // Control command publication topic
    if (!nh.getParam ("publish/cmd_sim", cmd_sim_) )
    {
        ROS_WARN(" Parameter 'output_cmd_sim' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // Robot state subscription topic
    if (!nh.getParam ("subscribe/robot_state", robot_state_) )
    {
        ROS_WARN(" Parameter 'robot_state_topic' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // Obstacle message subscription topic
    if (!nh.getParam ("subscribe/obstacles", ellipse_objects_feed_) )
    {
        ROS_WARN(" Parameter 'subscribe/obstacles' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // Pedestrians detected from SPencer Tracker message subscription topic
    if (!nh.getParam ("subscribe/pedestrians", pedestrians_objects_feed_) )
    {
        ROS_WARN(" Parameter 'subscribe/pedestrians' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    /** coordinate frame declarations **/
    // Robot coordinate frame
    if (!nh.getParam ("frames/robot_base_link", robot_base_link_) )
    {
        ROS_WARN(" Parameter 'robot_base_link' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // Planning coordinate frame
    if (!nh.getParam ("frames/planning_frame", planning_frame_) )
    {
        ROS_WARN(" Parameter 'planning_frame' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    /** Path parametrization setting **/
    // Number of segments in the local reference path
    if (!nh_config.getParam ("parametrization/n_local", n_local_) )
    {
        ROS_WARN(" Parameter '/parametrization/n_local not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Number of polynomials fitted per clothoid
    if (!nh_config.getParam ("parametrization/n_poly_per_clothoid", n_poly_per_clothoid_) )
    {
        ROS_WARN(" Parameter '/parametrization/n_poly_per_clothoid not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Number of search points in line search for path parameter
    if (!nh_config.getParam ("parametrization/n_search_points", n_search_points_) )
    {
        ROS_WARN(" Parameter '/parametrization/n_search_points not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Search window for the refined path parameter line search
    if (!nh_config.getParam ("parametrization/search_window_size", search_window_size_) )
    {
        ROS_WARN(" Parameter '/parametrization/search_window_size not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    /** predefined global reference path **/
    // Predefined path x-coordinates
    if (!nh_config.getParam ("global_path/x", ref_x_) )
    {
        ROS_WARN(" Parameter '/global_path/x not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Predefined path y-coordinates
    if (!nh_config.getParam ("global_path/y", ref_y_) )
    {
        ROS_WARN(" Parameter '/global_path/y not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Predefined path heading
    if (!nh_config.getParam ("global_path/theta", ref_theta_) )
    {
        ROS_WARN(" Parameter '/global_path/theta not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Reference velocity
    nh.param("global_path/reference_velocity", reference_velocity_, double(0.5)); // 0.5 by default

    /** Collision avoidance parameters **/
    // Number of dynamic obstacles
    if (!nh_config.getParam ("collision_avoidance/n_obstacles", n_obstacles_) )
    {
        ROS_WARN(" Parameter '/collision_avoidance/n_obstacles not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    /** OCP weight factors **/
    // Contouring control weight factors
    if (!nh_config.getParam ("ocp/weights/contour_weight_factors", contour_weight_factors_) )
    {
        ROS_WARN(" Parameter '/ocp/weights/contour_weight_factors' not set on %s node " ,
                 ros::this_node::getName().c_str());
        contour_weight_factors_.resize(2, 5.0);

        for (int i = 0u; i < contour_weight_factors_.size(); ++i)
        {
            ROS_INFO("Default contour weight factors value %f", contour_weight_factors_.at(i));
        }
    }

    // Control weight factors
    if (!nh_config.getParam ("ocp/weights/control_weight_factors", control_weight_factors_) )
    {
        ROS_WARN(" Parameter '/ocp/weights/control_weight_factors' not set on %s node " ,
                 ros::this_node::getName().c_str());
        // same as degree of freedom
        control_weight_factors_.resize(control_dim_, 1.0);

        for (int i = 0u; i < control_weight_factors_.size(); ++i)
        {
            ROS_INFO("Default control weight factors value %f", control_weight_factors_.at(i));
        }
    }

    // Repulsion weight factor
    nh.param("ocp/weights/repulsive_weight", repulsive_weight_, double(0.2)); // 0.1 by default
    // slack weight factor
    nh.param("ocp/weights/slack_weight", slack_weight_, double(1000.0)); // 1000 by default


    /** OCP constraint values **/
    // Minimum velocity constraints
    if (!nh_config.getParam ("ocp/constraints/velocity_constraints/min", vel_min_limit_) )
    {
        ROS_WARN(" Parameter '/ocp/constraints/velocity_constraints/min' not set on %s node" , ros::this_node::getName().c_str());
        //constraining vx vy and w
        vel_min_limit_.resize(3, -1.0);

        for (int i = 0u; i < 3; ++i)
        {
            ROS_INFO("Velocity default min limit value %f", vel_min_limit_.at(i));
        }
    }

    // Maximum velocity constraints
    if (!nh_config.getParam ("ocp/constraints/velocity_constraints/max", vel_max_limit_) )
    {
        ROS_WARN(" Parameter '/ocp/constraints/velocity_constraints/max' not set on %s node " ,  ros::this_node::getName().c_str());
        vel_max_limit_.resize(3, 1.0);

        for (int i = 0u; i < 3 ; ++i)
        {
            ROS_INFO("Velocity default min limit value %f", vel_max_limit_.at(i));
        }
    }

    /** ACADO configuration **/
    nh_config.param("acado_config/max_num_iteration", max_num_iteration_, int(10));  // maximum number of iteration for slution of OCP
    nh_config.param("acado_config/kkt_tolerance", kkt_tolerance_, double(1e-3));  // kkt condition for optimal solution
    nh_config.param("acado_config/integrator_tolerance", integrator_tolerance_, double(1e-3));  // intergrator tolerance
    nh_config.param("acado_config/time_horizon", time_horizon_, double(5.0));  // start time horizon for defining OCP problem
    nh_config.param("acado_config/discretization_intervals", discretization_intervals_, int(50));  // discretization_intervals for slution of OCP

    initialize_success_ = true;

    if (activate_debug_output_)
    {
        print_configuration_parameter();
    }

    ROS_WARN("LMPCC PARAMETERS INITIALIZED");

    return true;
}

// update configuration parameter
bool LMPCC_configuration::updateConfiguration(const LMPCC_configuration &new_config)
{
    initialize_success_ = new_config.initialize_success_;

    simulation_mode_ = new_config.simulation_mode_;
    gazebo_simulation_ = new_config.gazebo_simulation_;

    activate_output_ = new_config.activate_output_;
    activate_debug_output_ = new_config.activate_debug_output_;
    activate_timing_output_ = new_config.activate_timing_output_;
    activate_visualization_ = new_config.activate_visualization_;
    activate_feedback_message_ = new_config.activate_feedback_message_;

    controller_frequency_ = new_config.controller_frequency_;

    n_discs_ = new_config.n_discs_;
    ego_l_ = new_config.ego_l_;
    ego_w_ = new_config.ego_w_;

    state_dim_ = new_config.state_dim_;
    control_dim_ = new_config.control_dim_;

    cmd_ = new_config.cmd_;
    cmd_sim_ = new_config.cmd_sim_;
    robot_state_ = new_config.robot_state_;
    ellipse_objects_feed_ = new_config.ellipse_objects_feed_;

    robot_base_link_ = new_config.robot_base_link_;
    planning_frame_ = new_config.planning_frame_;

    n_local_ = new_config.n_local_;
    n_poly_per_clothoid_ = new_config.n_poly_per_clothoid_;
    n_search_points_ = new_config.n_search_points_;
    search_window_size_ = new_config.search_window_size_;

    ref_x_ = new_config.ref_x_;
    ref_y_ = new_config.ref_y_;
    ref_theta_ = new_config.ref_theta_;
    reference_velocity_ = new_config.reference_velocity_;

    contour_weight_factors_ = new_config.contour_weight_factors_;
    control_weight_factors_ = new_config.control_weight_factors_;
    repulsive_weight_ = new_config.repulsive_weight_;
    slack_weight_ = new_config.slack_weight_;

    vel_min_limit_ = new_config.vel_min_limit_;
    vel_max_limit_ = new_config.vel_max_limit_;

    max_num_iteration_ = new_config.max_num_iteration_;
    kkt_tolerance_ = new_config.kkt_tolerance_;
    integrator_tolerance_ = new_config.integrator_tolerance_;
    time_horizon_ = new_config.time_horizon_;
    discretization_intervals_ = new_config.discretization_intervals_;

    if (activate_debug_output_)
    {
        print_configuration_parameter();
    }

    return initialize_success_;
}

// print all data member of this class
void LMPCC_configuration::print_configuration_parameter()
{
    ROS_INFO_STREAM("Initialize_success: " << std::boolalpha << initialize_success_);

    ROS_INFO_STREAM("Activate output: " << std::boolalpha << activate_output_);
    ROS_INFO_STREAM("Activate debug output: " << std::boolalpha << activate_debug_output_);
    ROS_INFO_STREAM("Activate visualization: " << std::boolalpha << activate_visualization_);
    ROS_INFO_STREAM("Activate feedback message: " << std::boolalpha << activate_feedback_message_);

    ROS_INFO_STREAM("Local robot coordinate frame" << robot_base_link_);
    ROS_INFO_STREAM("Planning coordinate frame" << planning_frame_);

    ROS_INFO_STREAM("Controller frequency: " << controller_frequency_);
    ROS_INFO_STREAM("Max num iteration: " << max_num_iteration_);
    ROS_INFO_STREAM("KKT tolerance: " << kkt_tolerance_);
    ROS_INFO_STREAM("Integrator tolerance: " << integrator_tolerance_);
    ROS_INFO_STREAM("Time horizon: " << time_horizon_);
    ROS_INFO_STREAM("Discretization intervals: " << discretization_intervals_);

  // print joint vel min limit
  std::cout << "Joint vel min limit: [";
  for_each(vel_min_limit_.begin(), vel_min_limit_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print joint vel max limit
  std::cout << "Joint vel max limit: [";
  for_each(vel_max_limit_.begin(), vel_max_limit_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print contour weight factors
  std::cout << "Contour weight factors: [";
  for_each(contour_weight_factors_.begin(), contour_weight_factors_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

  // print control weight factors
  std::cout << "Control weight factors: [";
  for_each(control_weight_factors_.begin(), control_weight_factors_.end(), [](double& val)
  {
    std::cout << val << ", " ;
  }
  );
  std::cout<<"]"<<std::endl;

}

// clear allocated data from vector
void LMPCC_configuration::free_allocated_memory()
{
    vel_min_limit_.clear();
    vel_max_limit_.clear();

    ref_x_.clear();
    ref_y_.clear();
    ref_theta_.clear();

    contour_weight_factors_.clear();
    control_weight_factors_.clear();
}
