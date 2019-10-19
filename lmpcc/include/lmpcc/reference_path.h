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

// std includes
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <limits>
#include <memory>

// Include ROS
#include <ros/ros.h>

// eigen includes
#include <Eigen/Eigen>

// Include tkspline library
#include <tkspline/spline.h>

// Include clothoid library
#include <lmpcc/Clothoid.h>

class ReferencePath
{
    // Vectors containing coordinates of global path
    std::vector<double> X_global, Y_global, Theta_global;

    // Vectors containing coordinates of local path
    std::vector<double> S_local, X_local ,Y_local , V_local;

    // Number of segments that form the local reference path
    int N_segments;

    // Number of spline points in the local reference path
    int N_pts_spline;

    // Number of points in the local clothoid path
    int N_pts_clothoid;

    // Number of segments in each clothoid
    int N_seg_in_cloth;

    // Number of total clothoid segments in global path
    int N_cloth_segments;

    // Local path length
    double total_length;

    // Initial traveled distance at beginning of local reference path
    double s0;

    bool globalPathSet;

public:
    ReferencePath();

    ~ReferencePath();

    // Spline trajectories
    tk::spline ref_path_x, ref_path_y;

    /**
     * @brief SetGlobalPath: Initialize global reference path
     * @param X: X coordinates of the global path
     * @param Y: Y coordinates of the global path
     * @param Theta: Heading on the global path coordinates
     */
    void SetGlobalPath(std::vector<double> X, std::vector<double> Y, std::vector<double> Theta);

    /**
     * @brief PathSet: Check if global path is set
     * @return: Boolean indicating whether the path is set
     */
    bool PathSet(){return globalPathSet;};

    /**
     * @brief UpdateGlobalPath: Initialize global reference path
     * @param X: X coordinates of the global path
     * @param Y: Y coordinates of the global path
     * @param Theta: Heading on the global path coordinates
     */
    void UpdateGlobalPath(std::vector<double> X, std::vector<double> Y, std::vector<double> Theta);

    /**
     * @brief InitLocalRefPath: Initialize local reference path
     * @param X: X coordinates of the global path
     * @param Y: Y coordinates of the global path
     * @param Theta: Heading on the global path coordinates
     */
    void InitLocalRefPath(int N_local, int N_seg_per_cloth, std::vector<double> &s_local, std::vector<double> &x_local, std::vector<double> &y_local, std::vector<double> &v_local);

    /**
     * @brief UpdateLocalRefPath: Update local reference path
     * @param traj_i: Current segment the robot drives in
     */
    void UpdateLocalRefPath(int traj_i, std::vector<double> &S_local, std::vector<double> &x_local, std::vector<double> &y_local, std::vector<double> &v_local);

    /**
     * @brief ClosestPointOnPath: Find closest point on the local reference path by performing a line search around an initial guess
     * @param current_state: Current position of the robot
     * @param s_min
     * @param s_max
     * @param s_guess
     * @param window
     * @param n_tries
     */
    double ClosestPointOnPath(Eigen::Vector4d current_state, double s_min, double s_max, double s_guess, double window, int n_tries);

    /**
     * @brief SetPolynomialCoefficients: Store coefficients of the local reference path parametrization in a single vector
     * @param coefficientVector: Reference to the vector where the coefficients should be stored
 */
    void SetPolynomialCoefficients(std::vector<double> &coefficientVector);

    /**
     * @brief GetLocalX: Get the x coordinate of the local reference path spline at traveled distance s
     * @param s: Traveled distance s at which the local reference path should be accessed
     */
    double GetLocalX(double s){return ref_path_x(s);};

    /**
     * @brief GetLocalY: Get the x coordinate of the local reference path spline at traveled distance s
     * @param s: Traveled distance s at which the local reference path should be accessed
     */
    double GetLocalY(double s){return ref_path_y(s);};

    /**
     * @brief GetS0: Return initial traveled distance at beginning of local reference path
     * @return: s0
     */
    double GetS0(){return s0;};

    /**
     * @brief GetGlobalPath: Return the positions of the global path
     * @param x_global: reference to a vector to store the x coordinates in
     * @param y_global: reference to a vector to store the y coordinates in
     */
    void GetGlobalPath(std::vector<double> &x_global, std::vector<double> &y_global);

    /**
     * @brief GlobalPathLenght: Return number of points in the global reference pah
     * @return: N_global
     */
    int GlobalPathLenght(){return X_global.size();};

    /**
    * @brief PrintLocalPath: This function prints the global path in the terminal
    */
    void PrintLocalPath(std::vector<double> &s_local, std::vector<double> &x_local, std::vector<double> &y_local);

    /**
    * @brief PrintGlobalPath: This function prints the global path in the terminal
    */
    void PrintGlobalPath();
};

