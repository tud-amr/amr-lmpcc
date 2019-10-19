//
// Created by hai on 22/4/19.
//

#pragma once

// General include
#include <math.h>
#include <vector>

// Eigen includes
#include <Eigen/Dense>

// Ros include
#include <ros/ros.h>

// Message types
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

// Custom message includes. Auto-generated from msg/ directory.


// Define a class, including a constructor, member variables and member functions
class Obstacle_Prediction
{
public:
    //! Constructor, "main" will need to instantiate a ROS nodehandle, then pass it to the constructor
    explicit Obstacle_Prediction(ros::NodeHandle nh, std::string sub_topic, std::string pub_topic, double node_rate, double delta_t, int horizon_N);
    explicit Obstacle_Prediction(double delta_t, int horizon_N);

    nav_msgs::Path updateFilter(const geometry_msgs::Pose &msg);

private:
    //! Ros node handle
    ros::NodeHandle     nh_;        // we will need this, to pass between "main" and constructor

    //! Some objects to support subscriber, service, and publisher
    ros::Subscriber     sub_;
    ros::Publisher      pub_;
    double              node_rate_;             // node rate

    //! Obstacle measurement
    std::string         obstacle_sub_topic_;    // sub topic name from measurements (MoCap)
    Eigen::Vector3d     pos_measured_;          // measured position information

    //! Time information for filter
    ros::Time           time_stamp_;            // time stamp of current measurement
    ros::Time           time_stamp_previous_;   // time stamp of last measurement
    double              dt_;                    // time difference between two measurements

    //! Time information for predictor
    double              delta_t_;               // delta t of the predicted horizon
    int                 horizon_N_;             // prediction horizon length             

    //! Obstacle estimation and prediction
    std::string                 obstacle_pub_topic_;    // pub topic name after filtering and prediction
    Eigen::Matrix<double, 6, 1> state_estimated_;       // estimated state (pos & vel)
    Eigen::Matrix<double, 6, 6> state_cov_estimated_;   // estimated covariance matrix

    //! Initializations
    void initializeSubscribers();
    void initializePublishers();

    //! Subscriber callback
    void subscriberCallback(const geometry_msgs::PoseStamped &msg);


};
