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
 *          Boaz, Floor email:
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

#include <ros/ros.h>
#include <lmpcc/lmpcc_controller.h>

int main(int argc, char **argv)
{
  try
  {
    ros::init(argc, argv, ros::this_node::getName());
    LMPCC controller_;

    // initialize predictive control node
    if (!controller_.initialize())
    {
      ROS_ERROR_STREAM_NAMED("FAILED TO INITIALIZE %s", ros::this_node::getName().c_str());
      exit(1);
    }
    else
    {
      // spin node, till ROS node is running on
      ROS_INFO_STREAM_NAMED("%s INITIALIZE SUCCESSFULLY!!", ros::this_node::getName().c_str());
      ros::spin();
    }
  }

  catch (ros::Exception& e)
  {
    ROS_ERROR("predictive_control_node: Error occured: %s ", e.what());
    exit(1);
  }

  return 0;
}
