//
// Created by boaz on 10-10-18.
//

#include <static_collision_avoidance/static_environment.h>

StaticEnvironment::StaticEnvironment(){}

StaticEnvironment::~StaticEnvironment(){}

bool StaticEnvironment::LoadMap()
{

}

bool StaticEnvironment::initialize(){
    local_map_sub_ = nh_.subscribe("costmap_node/costmap/costmap", 1, &StaticEnvironment::LocalMapCallBack, this);
    local_map_updates_sub_ = nh_.subscribe("costmap_node/costmap/costmap_updates", 1, &StaticEnvironment::LocalMapUpdatesCallBack, this);
    pred_traj_sub_ = nh_.subscribe("predicted_trajectory", 1, &StaticEnvironment::PredictedTrajectoryCallback, this);

    collision_free_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("collision_free_area",1);
    collision_constraint_pub_ = nh_.advertise<static_collision_avoidance::collision_free_polygon>("collision_constraints",1);
    local_map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("my_map",1);

    /** Services **/
    map_service_ = nh_.serviceClient<nav_msgs::GetMap>("static_map");

    if (!nh_.getParam ("collision_avoidance/map_resolution", map_resolution_) )
    {
        ROS_WARN(" Parameter '/collision_avoidance/map_resolution not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    if (!nh_.getParam ("collision_avoidance/clean_ped_window_size", clean_ped_window_size_) )
    {
        ROS_WARN(" Parameter '/collision_avoidance/clean_ped_window_size not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    /** Collision avoidance parameters **/
    // Number of dynamic obstacles
    if (!nh_.getParam ("collision_avoidance/n_obstacles", n_obstacles_) )
    {
        ROS_WARN(" Parameter '/collision_avoidance/n_obstacles not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Occupation threshold of searched occupancy grid
    if (!nh_.getParam ("collision_avoidance/local_map", use_local_map_) )
    {
        ROS_WARN(" Parameter '/collision_avoidance/local_map not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Maxium search distance of area free of static obstacles
    if (!nh_.getParam ("collision_avoidance/delta_max", delta_max_) )
    {
        ROS_WARN(" Parameter '/collision_avoidance/delta_max not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Occupation threshold of searched occupancy grid
    if (!nh_.getParam ("collision_avoidance/free_space_assumption", free_space_assumption_) )
    {
        ROS_WARN(" Parameter '/collision_avoidance/free_space_assumption not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Occupation threshold of searched occupancy grid
    if (!nh_.getParam ("collision_avoidance/occupied_threshold", occupied_threshold_) )
    {
        ROS_WARN(" Parameter '/collision_avoidance/occupied_threshold not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Occupation threshold of searched occupancy grid
    if (!nh_.getParam ("state_dimension", state_dimension_) )
    {
        ROS_WARN(" Parameter '/state_dimension not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Occupation threshold of searched occupancy grid
    if (!nh_.getParam ("prediction_steps", prediction_steps_) )
    {
        ROS_WARN(" Parameter '/prediction_steps not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    // Occupation threshold of searched occupancy grid
    if (!nh_.getParam ("/collision_avoidance/delta_max", collision_free_delta_max_) )
    {
        ROS_WARN(" Parameter '/collision_avoidance/delta_max not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    if (!nh_.getParam ("activate_timing_output", activate_timing_output_) )
    {
        ROS_WARN(" Parameter '/activate_timing_output not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    if (!nh_.getParam ("frames/planning_frame", planning_frame_) )
    {
        ROS_WARN(" Parameter '/frames/planning_frame not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    cube1.type = visualization_msgs::Marker::CUBE;
    cube1.id = 60;
    cube1.color.r = 0.5;
    cube1.color.g = 0.5;
    cube1.color.b = 0.0;
    cube1.color.a = 0.1;
    cube1.header.frame_id = planning_frame_;
    cube1.ns = "trajectory";
    cube1.action = visualization_msgs::Marker::ADD;
    cube1.lifetime = ros::Duration(1);

    collision_free_C1.resize(prediction_steps_);
    collision_free_C2.resize(prediction_steps_);
    collision_free_C3.resize(prediction_steps_);
    collision_free_C4.resize(prediction_steps_);

    collision_free_a1x.resize(prediction_steps_);
    collision_free_a1y.resize(prediction_steps_);
    collision_free_a2x.resize(prediction_steps_);
    collision_free_a2y.resize(prediction_steps_);
    collision_free_a3x.resize(prediction_steps_);
    collision_free_a3y.resize(prediction_steps_);
    collision_free_a4x.resize(prediction_steps_);
    collision_free_a4y.resize(prediction_steps_);

    collision_free_xmin.resize(prediction_steps_);
    collision_free_xmax.resize(prediction_steps_);
    collision_free_ymin.resize(prediction_steps_);
    collision_free_ymax.resize(prediction_steps_);

    /** Request static environment map **/
    if (map_service_.call(map_srv_))
    {
        ROS_ERROR("Service GetMap succeeded.");
        global_map_ = map_srv_.response.map;
    }
    else
    {
        ROS_ERROR("Service GetMap failed.");
    }

    //ROS_INFO_STREAM("DONE INITIALIZING");

    return true;
}

void StaticEnvironment::PredictedTrajectoryCallback(const nav_msgs::Path msg){

    //ROS_INFO_STREAM("PredictedTrajectoryCallback");

    static_collision_avoidance::collision_free_polygon constraint_msg;
    pred_traj_ = msg;

    ComputeCollisionFreeArea();

    constraint_msg.collision_free_a1x = collision_free_a1x;
    constraint_msg.collision_free_a1y = collision_free_a1y;
    constraint_msg.collision_free_a2x = collision_free_a2x;
    constraint_msg.collision_free_a2y = collision_free_a2y;
    constraint_msg.collision_free_a3x = collision_free_a3x;
    constraint_msg.collision_free_a3y = collision_free_a3y;
    constraint_msg.collision_free_a4x = collision_free_a4x;
    constraint_msg.collision_free_a4y = collision_free_a4y;

    constraint_msg.collision_free_C1 = collision_free_C1;
    constraint_msg.collision_free_C2 = collision_free_C2;
    constraint_msg.collision_free_C3 = collision_free_C3;
    constraint_msg.collision_free_C4 = collision_free_C4;

    constraint_msg.collision_free_xmin = collision_free_xmin;
    constraint_msg.collision_free_xmax = collision_free_xmax;
    constraint_msg.collision_free_ymin = collision_free_ymin;
    constraint_msg.collision_free_ymax = collision_free_ymax;
    constraint_msg.computation_time = te_collision_free_;

    collision_constraint_pub_.publish(constraint_msg);

    publishPosConstraint();
}

int StaticEnvironment::getRotatedOccupancy(int x_i, int search_x, int y_i, int search_y, double psi) {
    //ROS_INFO_STREAM("getRotatedOccupancy");
    int x_search_rotated = (int) round(cos(psi) * search_x - sin(psi) * search_y);
    int y_search_rotated = (int) round(sin(psi) * search_x + cos(psi) * search_y);

    if ((x_i + x_search_rotated) > static_map_.info.width || (y_i + y_search_rotated) > static_map_.info.height ||
        (x_i + x_search_rotated) < 0 || (y_i + y_search_rotated) < 0) {
        return (int) 100;
    }
    else {
        return static_map_.data[static_map_.info.width * (y_i + y_search_rotated) + (x_i + x_search_rotated)];
    }
}

void StaticEnvironment::LocalMapCallBack(const nav_msgs::OccupancyGrid local_map)
{
    if (use_local_map_ ) {
        //ROS_WARN("StaticEnvironment::LocalMapCallBack");
        local_map_ = local_map;
        local_map_pub_.publish(local_map_);
        ROS_INFO("local map update received!");
    }
}

void StaticEnvironment::LocalMapUpdatesCallBack(const map_msgs::OccupancyGridUpdate local_map_update)
{

    if (use_local_map_ ) {
        ROS_ERROR("StaticEnvironment::LocalMapUpdatesCallBack");
        int index = 0;
        geometry_msgs::Pose obs;
        for (int y = local_map_update.y; y < local_map_update.y + local_map_update.height; y++) {
            for (int x = local_map_update.x; x < local_map_update.x + local_map_update.width; x++) {
                local_map_.data[local_map_.info.width * y + x] = local_map_update.data[index++];
            }
        }
        //ROS_INFO_STREAM("local_map_update.y: " << local_map_update.y << std::endl);
        //ROS_INFO_STREAM("local_map_update.x: " << local_map_update.x << std::endl);
        //ROS_INFO_STREAM("local_map_update.height: " << local_map_update.height << std::endl);
        //ROS_INFO_STREAM("local_map_update.height: " << local_map_update.height << std::endl);
        //ROS_INFO_STREAM("local_map_update.header.frame_id: " << local_map_update.header.frame_id << std::endl);

        if (clean_pedestrians_) {
            for (int total_obst_it = 0; total_obst_it < n_obstacles_; total_obst_it++) {
                //ROS_INFO_STREAM("obstacle id: " << total_obst_it << std::endl);
                /*obs.position.x = -current_state_(0) + obstacles_.StaticEnvironment_obstacles[total_obst_it].pose.position.x +local_map_update.width*map_resolution_/2;
                obs.position.y = -current_state_(1) + obstacles_.StaticEnvironment_obstacles[total_obst_it].pose.position.y +local_map_update.height*map_resolution_/2;

                //transformPose(planning_frame_,robot_base_link_,obs);
                int pos_x = obs.position.x/map_resolution_;
                int pos_y = obs.position.y/map_resolution_;

                //ROS_INFO_STREAM("Obstacle_pose_x: " << obstacles_.StaticEnvironment_obstacles[total_obst_it].pose.position.x << std::endl);
                //ROS_INFO_STREAM("Obstacle_pose_y: " << obstacles_.StaticEnvironment_obstacles[total_obst_it].pose.position.x << std::endl);
                //ROS_INFO_STREAM("Robot_pose_x: " << current_state_(0) << std::endl);
                //ROS_INFO_STREAM("Robot_pose_y: " << current_state_(1)<< std::endl);
                //ROS_INFO_STREAM("Obstacle_pose_map_x: " << obs.position.x << std::endl);
                //ROS_INFO_STREAM("Obstacle_pose_map_y: " << obs.position.y<< std::endl);

                //ROS_INFO_STREAM("x: " << pos_x << std::endl);
                int y_ini = std::min(std::max(0,pos_y-clean_ped_window_size_+clean_offset_y_),int(local_map_update.height));
                int y_end = std::max(std::min(int(local_map_update.height),pos_y+clean_ped_window_size_+clean_offset_y_),0);
                int x_ini = std::min(std::max(0,pos_x-clean_ped_window_size_+clean_offset_x_),int(local_map_update.width));
                int x_end = std::max(std::min(int(local_map_update.width),pos_x+clean_ped_window_size_+clean_offset_x_),0);
                for(int y = y_ini; y < y_end; y++)
                {
                    for(int x = x_ini; x < x_end; x++)
                    {
                        local_map_.data[ local_map_.info.width*y + x ] =pedestrian_occ_level_;
                    }
                }
                */
            }
        }
        local_map_pub_.publish(local_map_);
    }
//    ROS_INFO("local map update received!");
}

void StaticEnvironment::ComputeCollisionFreeArea()
{
    // Initialize timer
    //ROS_INFO_STREAM("ComputeCollisionFreeARea");

    auto start = std::chrono::steady_clock::now();

    int x_path_i, y_path_i;
    double x_path, y_path, psi_path, theta_search, r;
    std::vector<double> C_N;

    int search_steps = 10;

    if (use_local_map_ )
    {
        static_map_ = local_map_;
    }
    else
    {
        static_map_ = global_map_;
    }

    collision_free_delta_min_ = collision_free_delta_max_;

    // Iterate over points in prediction horizon to search for collision free circles
    for (int i = 0; i < prediction_steps_; i++)
    {

        // Current search point of prediction horizon
        x_path = pred_traj_.poses[i].pose.position.x;
        y_path = pred_traj_.poses[i].pose.position.y;
        psi_path = pred_traj_.poses[i].pose.orientation.z;

        // Find corresponding index of the point in the occupancy grid map
        x_path_i = (int) round((x_path - static_map_.info.origin.position.x)/static_map_.info.resolution);
        y_path_i = (int) round((y_path - static_map_.info.origin.position.y)/static_map_.info.resolution);

        // Compute the constraint
        computeConstraint(x_path_i,y_path_i,x_path, y_path, psi_path, i);

//        if (N_it == ACADO_N - 1)
//        {
//            ROS_INFO_STREAM("---------------------------------------------------------------------------");
//            ROS_INFO_STREAM("Searching last rectangle at x = " << x_path << " y = " << y_path << " psi = " << psi_path);
//        }

    }
    auto end = std::chrono::steady_clock::now();

    te_collision_free_ = double(std::chrono::duration_cast <std::chrono::milliseconds> (end-start).count());

    if (activate_timing_output_)
        ROS_INFO_STREAM("Free space solve time " << te_collision_free_  << " ms");
}

void StaticEnvironment::computeConstraint(int x_i, int y_i, double x_path, double y_path, double psi_path, int N)
{
    //ROS_INFO_STREAM("computeConstraint");
    // Initialize linear constraint normal vectors
    std::vector<double> t1(2, 0), t2(2, 0), t3(2, 0), t4(2, 0);

    // Declare search iterators
    int x_min, x_max, y_min, y_max;
    int search_x, search_y;
    int r_max_i_min, r_max_i_max;

    // define maximum search distance in occupancy grid cells, based on discretization
    r_max_i_min = (int) round(-collision_free_delta_max_ /static_map_.info.resolution);
    r_max_i_max = (int) round(collision_free_delta_max_/static_map_.info.resolution);

    // Initialize found rectangle values with maxium search distance
    x_min = r_max_i_min;
    x_max = r_max_i_max;
    y_min = r_max_i_min;
    y_max = r_max_i_max;

    // Initialize search distance iterator
    int search_distance = 2;
    // Initialize boolean that indicates whether the region has been found
    bool search_region = true;

    // Iterate until the region is found
    while (search_region)
    {
        // Only search in x_min direction if no value has been found yet
        if (x_min == r_max_i_min)
        {
            search_x = -search_distance;
            for (int search_y_it = std::max(-search_distance,y_min); search_y_it < std::min(search_distance,y_max); search_y_it++)
            {
                // Assign value if occupied cell is found
                if (getRotatedOccupancy(x_i, search_x, y_i, search_y_it, psi_path) > occupied_threshold_)
                {
                    x_min = search_x;
                }
            }
        } //else {ROS_INFO_STREAM("Already found x_min = " << x_min);}

        // Only search in x_max direction if no value has been found yet
        if (x_max == r_max_i_max)
        {
            search_x = search_distance;
            for (int search_y_it = std::max(-search_distance,y_min); search_y_it < std::min(search_distance,y_max); search_y_it++)
            {

                // Assign value if occupied cell is found
                if (getRotatedOccupancy(x_i, search_x, y_i, search_y_it, psi_path) > occupied_threshold_)
                {
                    x_max = search_x;
                }
            }
        } //else {ROS_INFO_STREAM("Already found x_max = " << x_max);}

        // Only search in y_min direction if no value has been found yet
        if (y_min == r_max_i_min)
        {
            search_y = -search_distance;
            for (int search_x_it = std::max(-search_distance,x_min); search_x_it < std::min(search_distance,x_max); search_x_it++)
            {

                // Assign value if occupied cell is found
                if (getRotatedOccupancy(x_i, search_x_it, y_i, search_y, psi_path) > occupied_threshold_)
                {
                    y_min = search_y;
                }
            }
        } //else {ROS_INFO_STREAM("Already found y_min = " << y_min);}

        // Only search in y_max direction if no value has been found yet
        if (y_max == r_max_i_max)
        {
            search_y = search_distance;
            for (int search_x_it = std::max(-search_distance,x_min); search_x_it < std::min(search_distance,x_max); search_x_it++)
            {
                // Assign value if occupied cell is found
                if (getRotatedOccupancy(x_i, search_x_it, y_i, search_y, psi_path) > occupied_threshold_)
                {
                    y_max = search_y;
                }
            }
        } //else {ROS_INFO_STREAM("Already found y_max = " << y_max);}

        // Increase search distance
        search_distance++;
        // Determine whether the search is finished
        search_region = (search_distance < r_max_i_max) && ( x_min == r_max_i_min || x_max == r_max_i_max || y_min == r_max_i_min || y_max == r_max_i_max );
    }

    // Assign the rectangle values
    collision_free_xmin[N] = x_min*static_map_.info.resolution + 0.35;
    collision_free_xmax[N] = x_max*static_map_.info.resolution - 0.35;
    collision_free_ymin[N] = y_min*static_map_.info.resolution + 0.35;
    collision_free_ymax[N] = y_max*static_map_.info.resolution - 0.35;

    std::vector<double> sqx(4,0), sqy(4,0);

    sqx[0] = x_path + cos(psi_path)*collision_free_xmin[N] - sin(psi_path)*collision_free_ymin[N];
    sqx[1] = x_path + cos(psi_path)*collision_free_xmin[N] - sin(psi_path)*collision_free_ymax[N];
    sqx[2] = x_path + cos(psi_path)*collision_free_xmax[N] - sin(psi_path)*collision_free_ymax[N];
    sqx[3] = x_path + cos(psi_path)*collision_free_xmax[N] - sin(psi_path)*collision_free_ymin[N];

    sqy[0] = y_path + sin(psi_path)*collision_free_xmin[N] + cos(psi_path)*collision_free_ymin[N];
    sqy[1] = y_path + sin(psi_path)*collision_free_xmin[N] + cos(psi_path)*collision_free_ymax[N];
    sqy[2] = y_path + sin(psi_path)*collision_free_xmax[N] + cos(psi_path)*collision_free_ymax[N];
    sqy[3] = y_path + sin(psi_path)*collision_free_xmax[N] + cos(psi_path)*collision_free_ymin[N];

    t1[0] = (sqx[1] - sqx[0])/sqrt((sqx[1] - sqx[0])*(sqx[1] - sqx[0]) + (sqy[1] - sqy[0])*(sqy[1] - sqy[0]));
    t2[0] = (sqx[2] - sqx[1])/sqrt((sqx[2] - sqx[1])*(sqx[2] - sqx[1]) + (sqy[2] - sqy[1])*(sqy[2] - sqy[1]));
    t3[0] = (sqx[3] - sqx[2])/sqrt((sqx[3] - sqx[2])*(sqx[3] - sqx[2]) + (sqy[3] - sqy[2])*(sqy[3] - sqy[2]));
    t4[0] = (sqx[0] - sqx[3])/sqrt((sqx[0] - sqx[3])*(sqx[0] - sqx[3]) + (sqy[0] - sqy[3])*(sqy[0] - sqy[3]));

    t1[1] = (sqy[1] - sqy[0])/sqrt((sqx[1] - sqx[0])*(sqx[1] - sqx[0]) + (sqy[1] - sqy[0])*(sqy[1] - sqy[0]));
    t2[1] = (sqy[2] - sqy[1])/sqrt((sqx[2] - sqx[1])*(sqx[2] - sqx[1]) + (sqy[2] - sqy[1])*(sqy[2] - sqy[1]));
    t3[1] = (sqy[3] - sqy[2])/sqrt((sqx[3] - sqx[2])*(sqx[3] - sqx[2]) + (sqy[3] - sqy[2])*(sqy[3] - sqy[2]));
    t4[1] = (sqy[0] - sqy[3])/sqrt((sqx[0] - sqx[3])*(sqx[0] - sqx[3]) + (sqy[0] - sqy[3])*(sqy[0] - sqy[3]));

    collision_free_a1x[N] = t1[1];
    collision_free_a2x[N] = t2[1];
    collision_free_a3x[N] = t3[1];
    collision_free_a4x[N] = t4[1];

    collision_free_a1y[N] = -t1[0];
    collision_free_a2y[N] = -t2[0];
    collision_free_a3y[N] = -t3[0];
    collision_free_a4y[N] = -t4[0];

    collision_free_C1[N] = sqx[0]*collision_free_a1x[N] + sqy[0]*collision_free_a1y[N];
    collision_free_C2[N] = sqx[1]*collision_free_a2x[N] + sqy[1]*collision_free_a2y[N];
    collision_free_C3[N] = sqx[2]*collision_free_a3x[N] + sqy[2]*collision_free_a3y[N];
    collision_free_C4[N] = sqx[3]*collision_free_a4x[N] + sqy[3]*collision_free_a4y[N];
}

int StaticEnvironment::getOccupancy(int x_i, int y_i)
{
    //ROS_INFO_STREAM("getOccupancy");
    return static_map_.data[static_map_.info.width*y_i + x_i];
}

void StaticEnvironment::publishPosConstraint(){

    ROS_INFO("StaticEnvironment::publishPosConstraint");
    visualization_msgs::MarkerArray collision_free;
    double x_center, y_center;

    for (int i = 0; i < prediction_steps_; i++)
    {
        cube1.scale.x = -collision_free_xmin[i] + collision_free_xmax[i];
        cube1.scale.y = -collision_free_ymin[i] + collision_free_ymax[i];
        cube1.scale.z = 0.01;

        // Find the center of the collision free area to be able to draw it properly
        x_center = collision_free_xmax[i] - (-collision_free_xmin[i] + collision_free_xmax[i])/2;
        y_center = collision_free_ymax[i] - (-collision_free_ymin[i] + collision_free_ymax[i])/2;

        // Assign center of cube
        cube1.pose.position.x = pred_traj_.poses[i].pose.position.x + cos(pred_traj_.poses[i].pose.orientation.z)*x_center - sin(pred_traj_.poses[i].pose.orientation.z)*y_center;
        cube1.pose.position.y = pred_traj_.poses[i].pose.position.y + sin(pred_traj_.poses[i].pose.orientation.z)*x_center + cos(pred_traj_.poses[i].pose.orientation.z)*y_center;

        cube1.id = 400+i;
        cube1.pose.orientation.x = 0;
        cube1.pose.orientation.y = 0;
        cube1.pose.orientation.z = pred_traj_.poses[i].pose.orientation.z;
        cube1.pose.orientation.w = 1;
        ZRotToQuat(cube1.pose);
        collision_free.markers.push_back(cube1);
    }

    collision_free_pub_.publish(collision_free);
}

void StaticEnvironment::ZRotToQuat(geometry_msgs::Pose& pose)
{
    //ROS_INFO_STREAM("ZRotToQuat");
    pose.orientation.w = cos(pose.orientation.z * 0.5);
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = sin(pose.orientation.z * 0.5);
}