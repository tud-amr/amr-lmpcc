//
// Created by hai on 22/4/19.
//


#include <lmpcc_obstacle_feed/obstacle_prediction.h>

// Constructor:  this will get called whenever an instance of this class is created
Obstacle_Prediction::Obstacle_Prediction(double delta_t, int horizon_N): delta_t_(delta_t), horizon_N_(horizon_N)
{
    ROS_INFO("In class constructor of Obstacle_Prediction");

    // Initialization the state covariance, this is important for starting the Kalman filter
    double cov_pos = 1^2;
    double cov_vel = 1^2;
    state_cov_estimated_.setZero();
    state_cov_estimated_(0, 0) = cov_pos;
    state_cov_estimated_(1, 1) = cov_pos;
    state_cov_estimated_(2, 2) = cov_pos;
    state_cov_estimated_(3, 3) = cov_vel;
    state_cov_estimated_(4, 4) = cov_vel;
    state_cov_estimated_(5, 5) = cov_vel;

    // Other initialization
    pos_measured_.setZero();
    state_estimated_.setZero();

    time_stamp_ = ros::Time::now();
    time_stamp_previous_ = ros::Time::now();

    // discrete time using in the filter, delta_t
//     dt_ = 1.0 / node_rate_;
}

Obstacle_Prediction::Obstacle_Prediction(ros::NodeHandle nh, std::string sub_topic, std::string pub_topic, double node_rate, double delta_t, int horizon_N): nh_(nh), obstacle_sub_topic_(sub_topic), obstacle_pub_topic_(pub_topic), node_rate_(node_rate), delta_t_(delta_t), horizon_N_(horizon_N)
{
    ROS_INFO("In class constructor of Obstacle_Prediction");

    // Initialization subscriber and publisher
    initializeSubscribers();
    initializePublishers();

    // Initialization the state covariance, this is important for starting the Kalman filter
    double cov_pos = 1^2;
    double cov_vel = 1^2;
    state_cov_estimated_.setZero();
    state_cov_estimated_(0, 0) = cov_pos;
    state_cov_estimated_(1, 1) = cov_pos;
    state_cov_estimated_(2, 2) = cov_pos;
    state_cov_estimated_(3, 3) = cov_vel;
    state_cov_estimated_(4, 4) = cov_vel;
    state_cov_estimated_(5, 5) = cov_vel;

    // Other initialization
    pos_measured_.setZero();
    state_estimated_.setZero();

    time_stamp_ = ros::Time::now();
    time_stamp_previous_ = ros::Time::now();

    // discrete time using in the filter, delta_t
//     dt_ = 1.0 / node_rate_;
}


// Set up subscribers
void Obstacle_Prediction::initializeSubscribers()
{
    ROS_INFO("Initializing subscribers");
    sub_ = nh_.subscribe(obstacle_sub_topic_, 1, &Obstacle_Prediction::subscriberCallback, this);
    ROS_INFO_STREAM(obstacle_sub_topic_);
}


// Set up publisher
void Obstacle_Prediction::initializePublishers()
{
    ROS_INFO("Initializing publishers");
    pub_ = nh_.advertise<nav_msgs::Path>(obstacle_pub_topic_, 1, true);
    ROS_INFO_STREAM(obstacle_pub_topic_);
}


// Subscriber callback function
void Obstacle_Prediction::subscriberCallback(const geometry_msgs::PoseStamped &msg)
{
    // the real work is done in this callback function
    // it wakes up every time a new message is published on obstacle_sub_topic_

    // for debugging
    // ROS_INFO("Filtering");

    // get measured position
    pos_measured_(0) = msg.pose.position.x;
    pos_measured_(1) = msg.pose.position.y;
    pos_measured_(2) = msg.pose.position.z;

    // current time stamp of the message
    time_stamp_      = msg.header.stamp;

    // time difference. If using the node_rate to derive, then comment the following lines
    dt_ = (time_stamp_ - time_stamp_previous_).toSec();   // sec

    // perform Kalman Filtering
    // matrix of state transition model
    Eigen::Matrix<double, 6, 6> A;
    A << 1, 0, 0, dt_, 0, 0,
         0, 1, 0, 0, dt_, 0,
         0, 0, 1, 0, 0, dt_,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;
    Eigen::Matrix<double, 6, 3> B;
    double dt_2 = 0.5*dt_*dt_;
    B << dt_2, 0, 0,
         0, dt_2, 0,
         0, 0, dt_2,
         0, 0, 0,
         0, 0, 0,
         0, 0, 0;

    // matrix of observation model
    Eigen::Matrix<double, 3, 6> H;
    H << 1, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0;

    // noise matrix
    double R_pos = 10E-4;
    Eigen::Matrix<double, 3, 3> R;      // observation noise covariance
    R << R_pos, 0, 0,
         0, R_pos, 0,
         0, 0, R_pos;
    double Q_acc = 1000;                // mean of noisy acceleration
    double Q_pos = Q_acc * dt_2;
    double Q_vel = Q_acc * dt_;
    Eigen::Matrix<double, 6, 6> Q;
    Q << Q_pos, 0, 0, 0, 0, 0,
         0, Q_pos, 0, 0, 0, 0,
         0, 0, Q_pos, 0, 0, 0,
         0, 0, 0, Q_vel, 0, 0,
         0, 0, 0, 0, Q_vel, 0,
         0, 0, 0, 0, 0, Q_vel;

    // prediction
    Eigen::Vector3d u(0, 0, 0);
    state_estimated_ = A*state_estimated_ + B*u;
    state_cov_estimated_ = A*state_cov_estimated_*A.transpose() + Q;

    // update
    Eigen::Vector3d pos_residual;       // pos estimation residual
    pos_residual = pos_measured_ - H * state_estimated_;
    Eigen::Matrix<double, 3, 3> S;
    S = H*state_cov_estimated_*H.transpose() + R;
    Eigen::Matrix<double, 6, 3> K;      // the Kalman gain matrix
    K = (state_cov_estimated_*H.transpose()) * S.inverse();
    // new estimated state
    state_estimated_ = state_estimated_ + K*pos_residual;
    // new covariance
    Eigen::Matrix<double, 6, 6> I;
    I.setIdentity();                    // I is an identity matrix
    state_cov_estimated_ = (I - K*H) * state_cov_estimated_;
    
    // Trajectory prediction and prepare published message
    nav_msgs::Path msg_pub;
    msg_pub.poses.resize(horizon_N_);
    msg_pub.header = msg.header;                        // save the header information, including current frame_id and time stamp
    // The first (0) stores the current estimated position
    msg_pub.poses[0].header = msg.header;
    msg_pub.poses[0].pose.position.x = state_estimated_(0);
    msg_pub.poses[0].pose.position.y = state_estimated_(1);
    msg_pub.poses[0].pose.position.z = state_estimated_(2);
    msg_pub.poses[0].pose.orientation = msg.pose.orientation;
    // The preform prediction based on constant velocity assumption, only position is predicted
    Eigen::Matrix<double, 6, 6> F;
    F << 1, 0, 0, delta_t_, 0, 0,
         0, 1, 0, 0, delta_t_, 0,
         0, 0, 1, 0, 0, delta_t_,
         0, 0, 0, 1, 0, 0,
         0, 0, 0, 0, 1, 0,
         0, 0, 0, 0, 0, 1;
    Eigen::Matrix<double, 6, 1> state_now;
    Eigen::Matrix<double, 6, 1> state_next;
    state_now = state_estimated_;
    for (int i = 1; i < horizon_N_; i++)
    {
        // predicted state
        state_next = F*state_now;
        // store into the published message
        msg_pub.poses[i].pose.orientation = msg.pose.orientation;       // orientation is not used in fact
        msg_pub.poses[i].pose.position.x = state_next(0);
        msg_pub.poses[i].pose.position.y = state_next(1);
        msg_pub.poses[i].pose.position.z = state_next(2);
        msg_pub.poses[i].header.frame_id = msg.header.frame_id;
        msg_pub.poses[i].header.seq      = msg_pub.poses[i-1].header.seq + 1;
        msg_pub.poses[i].header.stamp.sec= msg_pub.poses[i-1].header.stamp.sec + delta_t_;
        msg_pub.poses[i].header.stamp.nsec= msg_pub.poses[i-1].header.stamp.nsec + 10E9*delta_t_;
        // set next to be now
        state_now = state_next;
    }

    // publish the message
    pub_.publish(msg_pub);

    // set time
    time_stamp_previous_ = time_stamp_;
}
nav_msgs::Path Obstacle_Prediction::updateFilter(const geometry_msgs::Pose &msg)
{
    // the real work is done in this callback function
    // it wakes up every time a new message is published on obstacle_sub_topic_

    // for debugging
    // ROS_INFO("Filtering");

    // get measured position
    pos_measured_(0) = msg.position.x;
    pos_measured_(1) = msg.position.y;
    pos_measured_(2) = msg.position.z;

    // current time stamp of the message
    time_stamp_      = ros::Time::now();

    // time difference. If using the node_rate to derive, then comment the following lines
    dt_ = (time_stamp_ - time_stamp_previous_).toSec();   // sec

    // perform Kalman Filtering
    // matrix of state transition model
    Eigen::Matrix<double, 6, 6> A;
    A << 1, 0, 0, dt_, 0, 0,
            0, 1, 0, 0, dt_, 0,
            0, 0, 1, 0, 0, dt_,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
    Eigen::Matrix<double, 6, 3> B;
    double dt_2 = 0.5*dt_*dt_;
    B << dt_2, 0, 0,
            0, dt_2, 0,
            0, 0, dt_2,
            0, 0, 0,
            0, 0, 0,
            0, 0, 0;

    // matrix of observation model
    Eigen::Matrix<double, 3, 6> H;
    H << 1, 0, 0, 0, 0, 0,
            0, 1, 0, 0, 0, 0,
            0, 0, 1, 0, 0, 0;

    // noise matrix
    double R_pos = 10E-4;
    Eigen::Matrix<double, 3, 3> R;      // observation noise covariance
    R << R_pos, 0, 0,
            0, R_pos, 0,
            0, 0, R_pos;
    double Q_acc = 1000;                // mean of noisy acceleration
    double Q_pos = Q_acc * dt_2;
    double Q_vel = Q_acc * dt_;
    Eigen::Matrix<double, 6, 6> Q;
    Q << Q_pos, 0, 0, 0, 0, 0,
            0, Q_pos, 0, 0, 0, 0,
            0, 0, Q_pos, 0, 0, 0,
            0, 0, 0, Q_vel, 0, 0,
            0, 0, 0, 0, Q_vel, 0,
            0, 0, 0, 0, 0, Q_vel;

    // prediction
    Eigen::Vector3d u(0, 0, 0);
    state_estimated_ = A*state_estimated_ + B*u;
    state_cov_estimated_ = A*state_cov_estimated_*A.transpose() + Q;

    // update
    Eigen::Vector3d pos_residual;       // pos estimation residual
    pos_residual = pos_measured_ - H * state_estimated_;
    Eigen::Matrix<double, 3, 3> S;
    S = H*state_cov_estimated_*H.transpose() + R;
    Eigen::Matrix<double, 6, 3> K;      // the Kalman gain matrix
    K = (state_cov_estimated_*H.transpose()) * S.inverse();
    // new estimated state
    state_estimated_ = state_estimated_ + K*pos_residual;
    // new covariance
    Eigen::Matrix<double, 6, 6> I;
    I.setIdentity();                    // I is an identity matrix
    state_cov_estimated_ = (I - K*H) * state_cov_estimated_;

    // Trajectory prediction and prepare published message
    nav_msgs::Path msg_pub;
    msg_pub.poses.resize(horizon_N_);
    //msg_pub.header = msg.header;                        // save the header information, including current frame_id and time stamp
    // The first (0) stores the current estimated position
    //msg_pub.poses[0].header = msg.header;
    msg_pub.poses[0].pose.position.x = state_estimated_(0);
    msg_pub.poses[0].pose.position.y = state_estimated_(1);
    msg_pub.poses[0].pose.position.z = state_estimated_(2);
    msg_pub.poses[0].pose.orientation = msg.orientation;
    // The preform prediction based on constant velocity assumption, only position is predicted
    Eigen::Matrix<double, 6, 6> F;
    F << 1, 0, 0, delta_t_, 0, 0,
            0, 1, 0, 0, delta_t_, 0,
            0, 0, 1, 0, 0, delta_t_,
            0, 0, 0, 1, 0, 0,
            0, 0, 0, 0, 1, 0,
            0, 0, 0, 0, 0, 1;
    Eigen::Matrix<double, 6, 1> state_now;
    Eigen::Matrix<double, 6, 1> state_next;
    state_now = state_estimated_;

    state_now(3) = std::min(state_estimated_(3),2.0);
    state_now(3) = std::max(state_estimated_(3),-2.0);
    state_now(4) = std::min(state_estimated_(4),2.0);
    state_now(4) = std::max(state_estimated_(4),-2.0);
    state_now(5) = std::min(state_estimated_(5),2.0);
    state_now(5) = std::max(state_estimated_(5),-2.0);


    for (int i = 1; i < horizon_N_; i++)
    {
        // predicted state
        state_next = F*state_now;
        // store into the published message
        msg_pub.poses[i].pose.orientation = msg.orientation;       // orientation is not used in fact
        msg_pub.poses[i].pose.position.x = state_next(0);
        msg_pub.poses[i].pose.position.y = state_next(1);
        msg_pub.poses[i].pose.position.z = state_next(2);
        //msg_pub.poses[i].header.frame_id = msg.header.frame_id;
        msg_pub.poses[i].header.seq      = msg_pub.poses[i-1].header.seq + 1;
        msg_pub.poses[i].header.stamp.sec= msg_pub.poses[i-1].header.stamp.sec + delta_t_;
        msg_pub.poses[i].header.stamp.nsec= msg_pub.poses[i-1].header.stamp.nsec + 10E9*delta_t_;
        // set next to be now
        state_now = state_next;
    }

    // set time
    time_stamp_previous_ = time_stamp_;

    return msg_pub;
}