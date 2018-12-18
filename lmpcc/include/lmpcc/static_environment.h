//
// Created by boaz on 10-10-18.
//

#ifndef PROJECT_STATICENVIRONMENT_H
#define PROJECT_STATICENVIRONMENT_H

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

#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

class StaticEnvironment
{
    nav_msgs::OccupancyGrid staticMap;

    ComputeConstraint(int x_i, int y_i, double x_path, double y_path, double psi_path, int N);

    GetOccupancy(int x_i, int y_i);

    GetRotatedOccupancy(int x_i, int search_x, int y_i, int search_y, double psi);

public:

    StaticEnvironment();

    ~StaticEnvironment();

    LoadMap();

    ComputeCollisionFreeArea();
};

#endif //PROJECT_STATICENVIRONMENT_H
