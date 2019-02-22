#include <lmpcc_obstacle_feed/lmpcc_obstacle_feed.h>

ObstacleFeed::ObstacleFeed()
{
    this->reconfigure_server_.reset();
}

ObstacleFeed::~ObstacleFeed()
{
  clearDataMember();
}

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
        N_obstacles_ = lmpcc_obstacle_feed_config_->obstacle_threshold_;
        distance_ = lmpcc_obstacle_feed_config_->distance_threshold_;
        obstacle_size_ = lmpcc_obstacle_feed_config_->obstacle_size_;

        dt_ = lmpcc_obstacle_feed_config_->prediction_horizon_/lmpcc_obstacle_feed_config_->discretization_steps_;

        /** Initialize publisher of ellipsoid obstacles **/
        obstacles_pub = nh_.advertise<lmpcc_msgs::lmpcc_obstacle_array>(lmpcc_obstacle_feed_config_->pub_obstacles_,1);

        /** Initialize obstacle visualization **/
        if (lmpcc_obstacle_feed_config_->activate_visualization_) {
            visualize_obstacles_pub = nh_.advertise<visualization_msgs::MarkerArray>(lmpcc_obstacle_feed_config_->pub_obstacles_vis_,1);
            obst1_path_pub = nh_.advertise<nav_msgs::Path>("obst1_path",1);
            obst2_path_pub = nh_.advertise<nav_msgs::Path>("obst2_path",1);
        }

        /** Setting up dynamic_reconfigure server for the ObstacleFeedConfig parameters **/
        ros::NodeHandle nh_obstacle("lmpcc_obstacle_feed");
        reconfigure_server_.reset(new dynamic_reconfigure::Server<lmpcc_obstacle_feed::ObstacleFeedConfig>(reconfig_mutex_, nh_obstacle));
        reconfigure_server_->setCallback(boost::bind(&ObstacleFeed::reconfigureCallback,this,_1,_2));

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

            obstacles_.lmpcc_obstacles.resize(lmpcc_obstacle_feed_config_->obst_pose_x_.size());
            obstacles_.header.stamp = ros::Time::now();
            obstacles_.header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;

            for (int obst_it = 0; obst_it < obstacles_.lmpcc_obstacles.size(); obst_it++)
            {
                obstacles_.lmpcc_obstacles[obst_it].trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
                obstacles_.lmpcc_obstacles[obst_it].trajectory.header.stamp = ros::Time::now();
                obstacles_.lmpcc_obstacles[obst_it].trajectory.header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;

                obstacles_.lmpcc_obstacles[obst_it].pose.position.x = lmpcc_obstacle_feed_config_->obst_pose_x_.at(obst_it);
                obstacles_.lmpcc_obstacles[obst_it].pose.position.y = lmpcc_obstacle_feed_config_->obst_pose_y_.at(obst_it);
                obstacles_.lmpcc_obstacles[obst_it].pose.position.z = 0;
                obstacles_.lmpcc_obstacles[obst_it].pose.orientation.z = lmpcc_obstacle_feed_config_->obst_pose_heading_.at(obst_it);

                obstacles_.lmpcc_obstacles[obst_it].minor_semiaxis = lmpcc_obstacle_feed_config_->obst_dim_minor_.at(obst_it);
                obstacles_.lmpcc_obstacles[obst_it].major_semiaxis = lmpcc_obstacle_feed_config_->obst_dim_major_.at(obst_it);

                for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++)
                {
                    obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.stamp = ros::Time::now();
                    obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;
                    obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.x = obstacles_.lmpcc_obstacles[obst_it].pose.position.x + dt_*traj_it*lmpcc_obstacle_feed_config_->v_x_.at(obst_it);
                    obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.y = obstacles_.lmpcc_obstacles[obst_it].pose.position.y + dt_*traj_it*lmpcc_obstacle_feed_config_->v_y_.at(obst_it);;
                    obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.position.z = obstacles_.lmpcc_obstacles[obst_it].pose.position.z;

                    obstacles_.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose.orientation.z = obstacles_.lmpcc_obstacles[obst_it].pose.orientation.z;
                }
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

            obstacles_sub = nh_.subscribe(lmpcc_obstacle_feed_config_->sub_detections_, 1, &ObstacleFeed::detectionsCallback, this);
            pedestrians_sub = nh_.subscribe(lmpcc_obstacle_feed_config_->sub_pedestrians_, 1, &ObstacleFeed::pedestriansCallback, this);
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
    N_obstacles_ = config.N_obstacles;
    distance_ = config.distance_threshold;
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

void ObstacleFeed::pedestriansCallback(const spencer_tracking_msgs::TrackedPersons& person)
{
    //ROS_INFO_STREAM("Pedestrian callback!");

    std::vector<uint32_t> objectIDs;
    std::vector<double> objectDistances;
    double distance, volume;

    vision_msgs::Detection3DArray local_objects;
    lmpcc_msgs::lmpcc_obstacle_array ellipses;
    lmpcc_msgs::lmpcc_obstacle_array local_ellipses;
    lmpcc_msgs::lmpcc_obstacle ellipse;
    vision_msgs::Detection3D ped;
    ellipse.trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);

    for (int object_it = 0; object_it < person.tracks.size(); object_it++)
    {
//        // SHIFTING ALL OBSTACLES IN SPACE
//        objectArray_.objects[object_it].pose.position.y = objectArray_.objects[object_it].pose.position.y + 2;

        //ROS_INFO_STREAM("-- Compute distance of obstacle to robot: " );
        distance = 1;//sqrt(pow(person.tracks[object_it].pose.pose.position.x,2) + pow(person.tracks[object_it].pose.pose.position.y,2));
        ped.bbox.center.position.x = person.tracks[object_it].pose.pose.position.x;
        ped.bbox.center.position.y = person.tracks[object_it].pose.pose.position.y;
        ped.bbox.center.orientation.x = person.tracks[object_it].pose.pose.orientation.x;
        ped.bbox.center.orientation.y = person.tracks[object_it].pose.pose.orientation.y;
        ped.bbox.center.orientation.z = person.tracks[object_it].pose.pose.orientation.z;
        ped.bbox.center.orientation.w = person.tracks[object_it].pose.pose.orientation.w;
         //ROS_INFO_STREAM("-- Received # pedestrians: " << person.tracks.size());

        // If distance is smaller than defined bound, add to obstacles
        if (distance < distance_ ){
            local_objects.detections.push_back(ped);
            objectDistances.push_back(distance);
            //ROS_INFO_STREAM("-- Received # pedestrians: " << person.tracks.size());
        }
    }

    //ROS_INFO_STREAM("For all obstacles, fit an ellipse");

    for (int local_obst_it = 0; local_obst_it < local_objects.detections.size(); local_obst_it++)
    {
        //ROS_INFO_STREAM("FitElting lipse");
        ellipse = FitEllipse(local_objects.detections[local_obst_it],objectDistances[local_obst_it]);

        for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++)
        {
            ellipse.trajectory.poses[traj_it].header.stamp = ros::Time::now();
            ellipse.trajectory.poses[traj_it].header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;

            ellipse.trajectory.header.frame_id = lmpcc_obstacle_feed_config_->planning_frame_;
            ellipse.trajectory.poses[traj_it].pose.position.x = ellipse.pose.position.x + dt_*traj_it*person.tracks[local_obst_it].twist.twist.linear.x;
            ellipse.trajectory.poses[traj_it].pose.position.y = ellipse.pose.position.y + dt_*traj_it*person.tracks[local_obst_it].twist.twist.linear.y;

        }

        ellipses.lmpcc_obstacles.push_back(ellipse);
        //ROS_INFO_STREAM("-- Received # 5: " << person.tracks.size());
    }

    //ROS_INFO_STREAM("Order obstacles according to distance");
    OrderObstacles(ellipses);

    local_ellipses.lmpcc_obstacles.clear();

    //ROS_INFO_STREAM("Transform and add to local obstacles upto a defined bound");
    int n = std::min(int(ellipses.lmpcc_obstacles.size()),N_obstacles_);
    for (int ellipses_it = 0; ellipses_it < n; ellipses_it++)
    {

        ZRotToQuat(ellipses.lmpcc_obstacles[ellipses_it].pose);

        transformPose(lmpcc_obstacle_feed_config_->robot_frame_,lmpcc_obstacle_feed_config_->planning_frame_,ellipses.lmpcc_obstacles[ellipses_it].pose);

        QuatToZRot(ellipses.lmpcc_obstacles[ellipses_it].pose);
        for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++)
        {
            //ROS_INFO_STREAM("ZRotToQuat");
            ZRotToQuat(ellipses.lmpcc_obstacles[ellipses_it].trajectory.poses[traj_it].pose);
            //ROS_INFO_STREAM("transformPose");
            transformPose(lmpcc_obstacle_feed_config_->robot_frame_,lmpcc_obstacle_feed_config_->planning_frame_,ellipses.lmpcc_obstacles[ellipses_it].trajectory.poses[traj_it].pose);
            //ROS_INFO_STREAM("QuatToZRot");
            QuatToZRot(ellipses.lmpcc_obstacles[ellipses_it].trajectory.poses[traj_it].pose);
        }
        local_ellipses.lmpcc_obstacles.push_back(ellipses.lmpcc_obstacles[ellipses_it]);
    }

    for (int ellipses_it = n; ellipses_it < N_obstacles_ ; ellipses_it++)
    {
        ellipse.pose.position.x = 1000;
        ellipse.pose.position.y = 1000;
        for (int traj_it = 0; traj_it < lmpcc_obstacle_feed_config_->discretization_steps_; traj_it++)
        {
            ellipse.trajectory.poses[traj_it].pose.position.x = 1000;
            ellipse.trajectory.poses[traj_it].pose.position.y = 1000;
        }

        local_ellipses.lmpcc_obstacles.push_back(ellipse);
    }

    //ROS_INFO_STREAM("Publish and visualize obstacles" << n);
    if(local_ellipses.lmpcc_obstacles.size()>0){
        publishObstacles(local_ellipses);
        visualizeObstacles(local_ellipses);
    }
//    && local_obst_it < lmpcc_obstacle_feed_config_->obstacle_threshold_
//    ROS_INFO_STREAM("Received array of " << objectArray.objects.size() << " objects");
//    ROS_INFO_STREAM("Stored " << objectIDs.size() << " object identities");
//    ROS_INFO_STREAM("Stored " << ellipses.lmpcc_obstacles.size() << " object identities");
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
        ZRotToQuat(marker.pose);                                // Get Quaternion rotation
        marker.scale.x = 2*obstacles.lmpcc_obstacles[obst_it].major_semiaxis;     // Marker is specified by diameter, not radius!
        marker.scale.y = 2*obstacles.lmpcc_obstacles[obst_it].minor_semiaxis;
        marker.scale.z = 0.01;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        markerArray.markers.push_back(marker);
        for (int traj_it = 0; traj_it < obstacles.lmpcc_obstacles[obst_it].trajectory.poses.size(); traj_it++) {
            marker.id = obst_it * 100 + traj_it;                                    // Obstacle ID
            marker.pose = obstacles.lmpcc_obstacles[obst_it].trajectory.poses[traj_it].pose;
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

lmpcc_msgs::lmpcc_obstacle ObstacleFeed::FitEllipse(const vision_msgs::Detection3D& object, const double& distance)
{
    //ROS_INFO_STREAM("FitEllipse");
    lmpcc_msgs::lmpcc_obstacle ellipse;
    ellipse.trajectory.poses.resize(lmpcc_obstacle_feed_config_->discretization_steps_);
    ellipse.major_semiaxis = obstacle_size_; // sqrt(pow(object.dimensions.x,2) + pow(object.dimensions.y,2))/2;
    ellipse.minor_semiaxis = obstacle_size_; // sqrt(pow(object.dimensions.x,2) + pow(object.dimensions.y,2))/2;
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

bool ObstacleFeed::getTransform(const std::string& from, const std::string& to, Eigen::VectorXd& stamped_pose)
{
    bool transform = false;
    stamped_pose = Eigen::VectorXd(6);
    tf::StampedTransform stamped_tf;

    // make sure source and target frame exist
    if (tf_listener_.frameExists(to) && tf_listener_.frameExists(from))
    {
        try
        {
            // find transforamtion between souce and target frame
            tf_listener_.waitForTransform(from, to, ros::Time(0), ros::Duration(0.02));
            tf_listener_.lookupTransform(from, to, ros::Time(0), stamped_tf);

            // translation
            stamped_pose(0) = stamped_tf.getOrigin().x();
            stamped_pose(1) = stamped_tf.getOrigin().y();
            stamped_pose(2) = stamped_tf.getOrigin().z();

            // convert quternion to rpy
            tf::Quaternion quat(stamped_tf.getRotation().getX(),
                                                    stamped_tf.getRotation().getY(),
                                                    stamped_tf.getRotation().getZ(),
                                                    stamped_tf.getRotation().getW()
            );

            if (lmpcc_obstacle_feed_config_->activate_debug_output_)
            {
                std::cout << "\033[94m" << "getTransform:" << " qx:" << stamped_tf.getRotation().getX()
                                    << "qy:" << stamped_tf.getRotation().getY()
                                    << "qz:" << stamped_tf.getRotation().getZ()
                                    << "qw:" << stamped_tf.getRotation().getW() << "\033[0m" <<std::endl;
            }

            tf::Matrix3x3 quat_matrix(quat);
            quat_matrix.getRPY(stamped_pose(3), stamped_pose(4), stamped_pose(5));

            if (lmpcc_obstacle_feed_config_->activate_debug_output_)
            {
                std::cout << "\033[32m" << "getTransform:" << " roll:" << stamped_pose(3)
                                    << " pitch:" << stamped_pose(4)
                                    << " yaw:" << stamped_pose(5)
                                    << "\033[0m" <<std::endl;
            }

            transform = true;
        }
        catch (tf::TransformException& ex)
        {
            ROS_ERROR("MPCC::getTransform: %s", ex.what());
        }
    }

    else
    {
        ROS_WARN("MPCC::getTransform: '%s' or '%s' frame doesn't exist, pass existing frame",
                         from.c_str(), to.c_str());
    }

    return transform;
}
