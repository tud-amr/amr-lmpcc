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
 *          Boaz, Floor   email: boazfloor@gmail.com
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

#include <lmpcc/reference_path.h>

ReferencePath::ReferencePath()
{
    globalPathSet = false;
}

ReferencePath::~ReferencePath()
{

}

void ReferencePath::SetGlobalPath(std::vector<double> X, std::vector<double> Y, std::vector<double> Theta)
{
    // Check if all reference vectors are of the same length
    if (!( (X.size() == Y.size()) && (X.size() == Theta.size()) && (Y.size() == Theta.size()) ) )
    {
        ROS_ERROR("Reference path inputs should be of equal length");
    }

    // Resize global path vectors
    X_global.resize(X.size());
    Y_global.resize(Y.size());
    Theta_global.resize(Theta.size());

    // Set global path
    X_global = X;
    Y_global = Y;
    Theta_global = Theta;

    globalPathSet = true;

    ROS_INFO_STREAM("Initialized global path of length " << X_global.size());
}

void ReferencePath::InitLocalRefPath(int N_local, int N_seg_per_cloth, std::vector<double> &s_local, std::vector<double> &x_local, std::vector<double> &y_local, std::vector<double> &v_local)
{
    // Number of segments in local reference path
    N_segments = N_local;

    // Number of segments in each clothoid fitted through the global points
    N_seg_in_cloth = N_seg_per_cloth;

    // Compute how many clothoid segments and points should be build for the local reference path
    N_cloth_segments = ceil((double) N_local/N_seg_in_cloth);

    // Number of points defining the local spline +1 for continuous transition in beginning
    N_pts_spline = N_local + 2;
    // Number of spline points in the local clothoid path
    N_pts_clothoid = N_cloth_segments*N_seg_in_cloth + 1;

    // Initialize reference velocity setpoint
    v_local.resize(N_local);

    for (int v_it = 0; v_it < N_local; v_it++)
    {
        v_local[v_it] = 1;
    }

    // Initialize variables to generate clothoid local reference path
    double k, dk, L;

    std::vector<double> X_cloth, Y_cloth;
    std::vector<double> X_temp(N_pts_clothoid), Y_temp(N_pts_clothoid), S_temp(N_pts_clothoid);

    total_length = 0;

    // Fit clothoid segments and store intermediate coordinates temporarily
    for (int clothoid_it = 0; clothoid_it < N_cloth_segments; clothoid_it++) {
        Clothoid::buildClothoid(X_global[clothoid_it], Y_global[clothoid_it], Theta_global[clothoid_it],
                                X_global[clothoid_it + 1], Y_global[clothoid_it + 1], Theta_global[clothoid_it + 1], k, dk,
                                L);

        Clothoid::pointsOnClothoid(X_global[clothoid_it], Y_global[clothoid_it], Theta_global[clothoid_it], k, dk, L,
                                   N_seg_in_cloth + 1, X_cloth, Y_cloth);

        for (int clothoid_points_it = 0; clothoid_points_it < N_seg_per_cloth; clothoid_points_it++) {
            X_temp[clothoid_it * N_seg_in_cloth + clothoid_points_it] = X_cloth[clothoid_points_it];
            Y_temp[clothoid_it * N_seg_in_cloth + clothoid_points_it] = Y_cloth[clothoid_points_it];
            S_temp[clothoid_it * N_seg_in_cloth + clothoid_points_it] =
                    total_length + (clothoid_points_it) * L / N_seg_in_cloth;
        }

        total_length += L;
    }

    // Set last point
    X_temp[N_pts_clothoid - 1] = X_global[X_global.size() - 1];
    Y_temp[N_pts_clothoid - 1] = Y_global[X_global.size() - 1];
    S_temp[N_pts_clothoid - 1] = total_length;

    // Resize local reference path vectors
    s_local.resize(N_pts_clothoid);
    x_local.resize(N_pts_clothoid);
    y_local.resize(N_pts_clothoid);

    s0 = 0.5;

    // Set first entries of the local reference path
    s_local[0] = 0;
    x_local[0] = X_temp[0] - s0;
    y_local[0] = Y_temp[0];

    // Set local reference path values
    for (int local_points_it = 1; local_points_it < N_pts_spline; local_points_it++ ) {
        x_local[local_points_it] = X_temp[local_points_it - 1];
        y_local[local_points_it] = Y_temp[local_points_it - 1];
        s_local[local_points_it] = S_temp[local_points_it - 1] + s0;
    }

    total_length = s_local[N_pts_spline - 1];

    // Set local spline points
    ref_path_x.set_points(s_local, x_local);
    ref_path_y.set_points(s_local, y_local);

    ROS_INFO_STREAM("Local reference path initialized");
}

void ReferencePath::UpdateLocalRefPath(int traj_i, std::vector<double> &s_local, std::vector<double> &x_local, std::vector<double> &y_local, std::vector<double> &v_local)
{
    double s_local_start = s_local[1];

    // Shift local reference path segments one place
    for (int local_path_it = 0; local_path_it < N_pts_spline - 1; local_path_it++) {
        s_local[local_path_it] = s_local[local_path_it + 1] - s_local_start;
        x_local[local_path_it] = x_local[local_path_it + 1];
        y_local[local_path_it] = y_local[local_path_it + 1];
    }

    for (int v_it = 0; v_it < N_segments - 1; v_it++){
        v_local[v_it] = v_local[v_it + 1];
    }

    // Initialize variables to generate clothoid local reference path
    double k, dk, L;

    std::vector<double> X_cloth, Y_cloth;
    std::vector<double> X_temp(3), Y_temp(3), S_temp(3);

    total_length = 0;

    double xroad1, xroad2, yroad1, yroad2, thetaroad1, thetaroad2;

    int cloth_end_i = floor( ((double) (traj_i + N_segments - 1))/((double) N_seg_in_cloth) );
//    ROS_INFO_STREAM("cloth_end_i = " << cloth_end_i);

    // Deal with last sections of the reference path
    if (cloth_end_i + 2 > X_global.size() ) {
        xroad1 = X_global[X_global.size() - 1];
        xroad2 = xroad1 + 0.1;
        yroad1 = Y_global[X_global.size() - 1];
        yroad2 = yroad1;
        thetaroad1 = Theta_global[X_global.size() - 1];
        thetaroad2 = thetaroad1;
        v_local[N_segments - 1] = 0;
    }
    else {
        xroad1 = X_global[cloth_end_i];
        xroad2 = X_global[cloth_end_i + 1];
        yroad1 = Y_global[cloth_end_i];
        yroad2 = Y_global[cloth_end_i + 1];
        thetaroad1 = Theta_global[cloth_end_i];
        thetaroad2 = Theta_global[cloth_end_i + 1];
        v_local[N_segments - 1] = 1;
    }

    // Build clothoid for extending the local reference path
    Clothoid::buildClothoid(xroad1, yroad1, thetaroad1, xroad2, yroad2, thetaroad2, k, dk,L);
    Clothoid::pointsOnClothoid(xroad1, yroad1, thetaroad1, k, dk, L, N_seg_in_cloth + 1, X_cloth, Y_cloth);

    X_temp[0] = X_cloth[0];
    Y_temp[0] = Y_cloth[0];
    S_temp[0] = 0;

    X_temp[1] = X_cloth[1];
    Y_temp[1] = Y_cloth[1];
    S_temp[1] = L/2;

    X_temp[2] = xroad2;
    Y_temp[2] = yroad2;
    S_temp[2] = L;

    s_local[N_pts_spline - 1] = s_local[N_pts_spline - 2] + S_temp[traj_i%N_seg_in_cloth + 1] - S_temp[traj_i%N_seg_in_cloth];
    x_local[N_pts_spline - 1] = X_temp[traj_i%N_seg_in_cloth + 1];
    y_local[N_pts_spline - 1] = Y_temp[traj_i%N_seg_in_cloth + 1];

    // Set local spline points
    ref_path_x.set_points(s_local,x_local);
    ref_path_y.set_points(s_local,y_local);

    total_length = s_local[N_pts_spline - 1];
    s0 = s_local[1];
}

double ReferencePath::ClosestPointOnPath(Eigen::Vector4d current_state, double s_min, double s_max, double s_guess, double window, int n_tries){

    double lower = std::max(s_min, s_guess-window);
    double upper = std::min(s_max, s_guess + window);
    double s_i=lower,spline_pos_x_i,spline_pos_y_i;
    double dist_i,min_dist,smin;

    spline_pos_x_i = ref_path_x(s_i);
    spline_pos_y_i = ref_path_y(s_i);

    min_dist = std::sqrt((spline_pos_x_i-current_state(0))*(spline_pos_x_i-current_state(0))+(spline_pos_y_i-current_state(1))*(spline_pos_y_i-current_state(1)));

    for(int i=0;i<n_tries;i++){
        s_i = lower+(upper-lower)/n_tries*i;
        spline_pos_x_i = ref_path_x(s_i);
        spline_pos_y_i = ref_path_y(s_i);
        dist_i = std::sqrt((spline_pos_x_i-current_state(0))*(spline_pos_x_i-current_state(0))+(spline_pos_y_i-current_state(1))*(spline_pos_y_i-current_state(1)));

        if(dist_i<min_dist){
            min_dist = dist_i;
            smin = s_i;
        }

        ros::Duration(1E-20).sleep();
    }
    if(smin < lower){
        smin=lower;
    }

    return smin;
}

void ReferencePath::GetGlobalPath(std::vector<double> &x_global, std::vector<double> &y_global)
{
    x_global = X_global;
    y_global = Y_global;
}

void ReferencePath::PrintLocalPath(std::vector<double> &s_local, std::vector<double> &x_local, std::vector<double> &y_local)
{
    ROS_INFO_STREAM("----- LOCAL PATH ------");
    for (int path_it = 1; path_it < x_local.size(); path_it++)
    {
        ROS_INFO_STREAM("s: " << s_local[path_it] << " x: " << x_local[path_it] << " y: " << y_local[path_it]);
    }
    ROS_INFO_STREAM("-----------------------");
}

void ReferencePath::PrintGlobalPath()
{
    ROS_INFO_STREAM("----- GLOBAL PATH ------");
    for (int path_it = 0; path_it < X_global.size(); path_it++)
    {
        ROS_INFO_STREAM("x: " << X_global[path_it] << " y: " << Y_global[path_it] << " theta: " << Theta_global[path_it]);
    }
    ROS_INFO_STREAM("------------------------");
}
