#ifndef LMPCC_OBSTACLE_FEED_H
#define LMPCC_OBSTACLE_FEED_H

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>

#include <std_srvs/Empty.h>

#include <Eigen/Dense>

#include <vector>

//Dynamic Reconfigure server
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <lmpcc_obstacle_feed/ObstacleFeedConfig.h>

#include <vision_msgs/Detection3DArray.h>
#include <vision_msgs/Detection3D.h>

// custom obstacle messages
#include <lmpcc_msgs/lmpcc_obstacle.h>
#include <lmpcc_msgs/lmpcc_obstacle_array.h>

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// configuration
#include <lmpcc_obstacle_feed/lmpcc_obstacle_feed_configuration.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


bool CompareObstacleDistance(lmpcc_msgs::lmpcc_obstacle const &obst1, lmpcc_msgs::lmpcc_obstacle const &obst2);

class ObstacleFeed
{
public:

    ObstacleFeed();

    ~ObstacleFeed();

    bool initialize();

    ros::Subscriber obstacles_sub, optitrack_sub;

    ros::Publisher obstacles_pub, visualize_obstacles_pub;
    ros::Publisher obst1_path_pub, obst2_path_pub;

    std_srvs::Empty emptyCall;
    ros::ServiceServer update_service;

    ros::Timer loop_timer;

    boost::shared_ptr<dynamic_reconfigure::Server<lmpcc_obstacle_feed::ObstacleFeedConfig> > reconfigure_server_;
    boost::recursive_mutex reconfig_mutex_;

    void reconfigureCallback(lmpcc_obstacle_feed::ObstacleFeedConfig& config, uint32_t level);

private:

    ros::NodeHandle nh_;

    tf::TransformListener tf_listener_;

    bool activate_debug_output_;

    boost::shared_ptr<lmpcc_obstacle_feed_configuration> lmpcc_obstacle_feed_config_;

    vision_msgs::Detection3DArray objectArray_;

    double maxV_, minV_, distance_, obstacle_threshold_, obstacle_size_;
    double dt_;
    int N_obstacles_;

    lmpcc_msgs::lmpcc_obstacle_array obstacles_;

    void spinNode();
    void clearDataMember();
    void detectionsCallback(const vision_msgs::Detection3DArray& objects);
    void optitrackCallback(const nav_msgs::Path& predicted_path);
    void updateObstacles(const ros::TimerEvent& event);
    void publishObstacles(const lmpcc_msgs::lmpcc_obstacle_array& obstacles);
    void visualizeObstacles(const lmpcc_msgs::lmpcc_obstacle_array& obstacles);
    void OrderObstacles(lmpcc_msgs::lmpcc_obstacle_array& ellipses);
    bool UpdateCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response);

    lmpcc_msgs::lmpcc_obstacle FitEllipse(const vision_msgs::Detection3D& object, const double& distance);
    void QuatToZRot(geometry_msgs::Pose& pose);
    void ZRotToQuat(geometry_msgs::Pose& pose);
    bool getTransform(const std::string& from, const std::string& to, Eigen::VectorXd& transform);
    bool transformPose(const std::string& from, const std::string& to, geometry_msgs::Pose& pose);

};

#endif // LMPCC_OBSTACLE_FEED_H
