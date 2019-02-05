#include <lmpcc_obstacle_feed/lmpcc_obstacle_feed_configuration.h>

lmpcc_obstacle_feed_configuration::lmpcc_obstacle_feed_configuration()
{
    initialize_success_ = false;
}

lmpcc_obstacle_feed_configuration::~lmpcc_obstacle_feed_configuration()
{
    free_allocated_memory();
}

bool lmpcc_obstacle_feed_configuration::initialize()
{
    ros::NodeHandle nh_config;
    ros::NodeHandle nh;

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("obstacle_feed_mode", obstacle_feed_mode_) )
    {
        ROS_WARN(" Parameter 'obstacle_feed_mode_' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("activate_debug_output", activate_debug_output_) )
    {
        ROS_WARN(" Parameter 'activate_debug_output' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("activate_visualization", activate_visualization_) )
    {
        ROS_WARN(" Parameter 'activate_visualization' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("update_rate", update_rate_) )
    {
        ROS_WARN(" Parameter 'update_rate' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("discretization_steps", discretization_steps_) )
    {
        ROS_WARN(" Parameter 'discretization_steps' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("prediction_horizon", prediction_horizon_) )
    {
        ROS_WARN(" Parameter 'prediction_horizon' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    /** Coordinate frames **/
    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("frames/planning_frame", planning_frame_) )
    {
      ROS_WARN(" Parameter '/frames/planning_frame' not set on %s node " , ros::this_node::getName().c_str());
      return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("frames/robot_frame", robot_frame_) )
    {
        ROS_WARN(" Parameter '/frames/robot_frame' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    /** ROS publishers and subscribers **/
    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("publish/obstacles", pub_obstacles_) )
    {
        ROS_WARN(" Parameter '/publish/obstacles' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("publish/obstacles_vis", pub_obstacles_vis_) )
    {
        ROS_WARN(" Parameter '/publish/obstacles_vis' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("subscribe/detections", sub_detections_) )
    {
        ROS_WARN(" Parameter '/subscribe/detections' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("subscribe/optitrack", sub_optitrack_) )
    {
        ROS_WARN(" Parameter '/subscribe/optitrack' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("subscribe/pedestrians", sub_pedestrians_) )
    {
        ROS_WARN(" Parameter '/subscribe/pedestrians' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    /** Detected obstacles criteria **/
    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("detected_obstacles/distance_threshold", distance_threshold_) )
    {
        ROS_WARN(" Parameter '/detected_obstacles/distance_threshold' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("detected_obstacles/obstacle_threshold", obstacle_threshold_) )
    {
        ROS_WARN(" Parameter '/detected_obstacles/obstacle_threshold' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("detected_obstacles/min_obstacle_volume", min_obstacle_volume_) )
    {
        ROS_WARN(" Parameter '/detected_obstacles/min_obstacle_volume' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("detected_obstacles/max_obstacle_volume", max_obstacle_volume_) )
    {
        ROS_WARN(" Parameter '/detected_obstacles/max_obstacle_volume' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    // read parameter from parameter server if not set than terminate code, as this parameter is essential parameter
    if (!nh.getParam ("detected_obstacles/obstacle_size", obstacle_size_) )
    {
        ROS_WARN(" Parameter '/detected_obstacles/obstacle_size' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    if (!nh.getParam ("detected_obstacles/lambda", lambda_) )
    {
        ROS_WARN(" Parameter '/detected_obstacles/lambda' not set on %s node " , ros::this_node::getName().c_str());
        return false;
    }

    /** Predefined obstacle parameters **/
    if (!nh_config.getParam ("predefined_obstacles/pose_x", obst_pose_x_) )
    {
        ROS_WARN(" Parameter '/predefined_obstacles/pose_x not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    if (!nh_config.getParam ("predefined_obstacles/pose_y", obst_pose_y_) )
    {
        ROS_WARN(" Parameter '/predefined_obstacles/pose_y not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    if (!nh_config.getParam ("predefined_obstacles/heading", obst_pose_heading_) )
    {
        ROS_WARN(" Parameter '/predefined_obstacles/pose_x not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    if (!nh_config.getParam ("predefined_obstacles/v_x", v_x_) )
    {
        ROS_WARN(" Parameter '/predefined_obstacles/v_x not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    if (!nh_config.getParam ("predefined_obstacles/v_y", v_y_) )
    {
        ROS_WARN(" Parameter '/predefined_obstacles/v_y not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    if (!nh_config.getParam ("predefined_obstacles/dimensions/minor_semiaxis", obst_dim_minor_) )
    {
        ROS_WARN(" Parameter '/predefined_obstacles/dimensions/minor_semiaxis not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    if (!nh_config.getParam ("predefined_obstacles/dimensions/major_semiaxis", obst_dim_major_) )
    {
        ROS_WARN(" Parameter '/predefined_obstacles/dimensions/major_semiaxis not set on %s node" , ros::this_node::getName().c_str());
        return false;
    }

    ROS_WARN("OBSTACLE FEED PARAMETER INITIALIZED");
    return true;
}

void lmpcc_obstacle_feed_configuration::free_allocated_memory()
{

}
