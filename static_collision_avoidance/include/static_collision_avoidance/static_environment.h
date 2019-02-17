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
#include <map_msgs/OccupancyGridUpdate.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>

// Visualization messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <chrono>

#include <static_collision_avoidance/collision_free_polygon.h>
class StaticEnvironment
{
    ros::NodeHandle nh_;

    nav_msgs::OccupancyGrid staticMap;

public:

    StaticEnvironment();

    ~StaticEnvironment();

    bool LoadMap();

    ros::ServiceClient map_service_;

    /* Variables */

    // subscriber for obstacle feed
    ros::Subscriber local_map_sub_;
    // subscriber for obstacle feed
    ros::Subscriber local_map_updates_sub_,pred_traj_sub_;
    ros::Publisher local_map_pub_,collision_free_pub_,collision_constraint_pub_;

    nav_msgs::OccupancyGrid global_map_, local_map_, static_map_;

    nav_msgs::GetMap map_srv_;

    bool clean_pedestrians_;
    int clean_offset_x_;
    int clean_offset_y_;
    int pedestrian_occ_level_;

    double map_resolution_;
    int clean_ped_window_size_;
    int n_obstacles_;

    double te_collision_free_;

    int prediction_steps_;
    int state_dimension_;

    visualization_msgs::Marker cube1;

    std::string planning_frame_;

    nav_msgs::Path pred_traj_;

    bool activate_timing_output_;

    /** Number of dynamic obstacles **/
    bool use_local_map_;
    double collision_free_delta_max_, collision_free_delta_min_;
    double delta_max_;
    bool free_space_assumption_;
    int occupied_threshold_;

    std::vector<double> collision_free_C1, collision_free_C2, collision_free_C3, collision_free_C4, collision_free_a1x ,collision_free_a1y, collision_free_a2x ,collision_free_a2y, collision_free_a3x ,collision_free_a3y, collision_free_a4x ,collision_free_a4y , collision_free_xmin, collision_free_xmax, collision_free_ymin, collision_free_ymax;

    /**
    * @brief getRotatedOccupancy: Returns the occupancy cell value at specified index, for a rotated map
    * @param x_i: Index of grid cell in x direction
    * @param y_i: Index of grid cell in y direction
    * @param psi: Rotation of the map
    */
    int getRotatedOccupancy(int x_i, int search_x, int y_i, int search_y, double psi);

    bool initialize();

    void LocalMapCallBack(const nav_msgs::OccupancyGrid local_map);

    void PredictedTrajectoryCallback(const nav_msgs::Path msg);

    void LocalMapUpdatesCallBack(const map_msgs::OccupancyGridUpdate local_map_update);

    /**
     * @brief ComputeCollisionFreeArea: Compute the collision free are around the prediction horizon, approximated by circles located at each discretization step
     */
    void ComputeCollisionFreeArea();

    /**
     * @brief searchRadius: Find the true radius from an occupancy grid index to the first occupied cell
     * @param x_i: Index of grid cell in x direction
     * @param y_i: Index of grid cell in y direction
     */
    void computeConstraint(int x_i, int y_i, double x_path, double y_path, double psi_path, int N);

    /**
     * @brief getOccupancy: Returns the occupancy cell value at specified index
     * @param x_i: Index of grid cell in x direction
     * @param y_i: Index of grid cell in y direction
     */
    int getOccupancy(int x_i, int y_i);

    /**
    * @brief publishPosConstraint: Publish the approximated collision free area as a markerarray
    */
    void publishPosConstraint();

    void ZRotToQuat(geometry_msgs::Pose& pose);
};

#endif //PROJECT_STATICENVIRONMENT_H
