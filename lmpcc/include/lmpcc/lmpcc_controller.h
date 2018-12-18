#ifndef LMPCC_CONTROLLER_H
#define LMPCC_CONTROLLER_H

// ros includes
#include <pluginlib/class_loader.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/JointState.h>
#include <tf/tf.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>

// eigen includes
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>

// Include pre-generated OCP
#include <acado_common.h>
#include <acado_auxiliary_functions.h>

// std includes
#include <iostream>
#include <string>
#include <vector>
#include <math.h>
#include <algorithm>
#include <limits>

// boost includes
#include <boost/shared_ptr.hpp>
#include <boost/scoped_ptr.hpp>

// Visualization messages
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/InteractiveMarker.h>
#include <visualization_msgs/InteractiveMarkerControl.h>

// yaml parsing
#include <fstream>
#include <yaml-cpp/yaml.h>

// lmpcc configuration
#include <lmpcc/lmpcc_configuration.h>

// reference path class
#include <lmpcc/reference_path.h>

// actions, srvs, msgs
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <lmpcc/moveAction.h>
#include <lmpcc/moveActionGoal.h>
//#include <lmpcc/collision_avoidance.h>

// joint trajectory interface
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <lmpcc/trajAction.h>
#include <lmpcc/trajActionGoal.h>

// navigation messages
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

// lmpcc messages
#include <lmpcc_msgs/lmpcc_feedback.h>
#include <lmpcc_msgs/lmpcc_obstacle.h>
#include <lmpcc_msgs/lmpcc_obstacle_array.h>

//Dynamic Reconfigure server
#include <boost/thread/mutex.hpp>
#include <boost/shared_ptr.hpp>
#include <dynamic_reconfigure/server.h>
#include <lmpcc/LmpccConfig.h>

//TF
#include <tf2_ros/transform_broadcaster.h>

//Joint states
#include <sensor_msgs/JointState.h>

//splines
#include <tkspline/spline.h>
#include <lmpcc/Clothoid.h>


typedef double real_t;

class LMPCC
{
    /** Managing execution of all classes of lmpcc
     * - Handle self collision avoidance
     * - Extract current position and velocity of manipulator joints
     * - Publish controlled joint velocity
     */
public:

    // Dynamic reconfigure server
    boost::shared_ptr<dynamic_reconfigure::Server<lmpcc::LmpccConfig> > reconfigure_server_;
    boost::recursive_mutex reconfig_mutex_;
    void reconfigureCallback(lmpcc::LmpccConfig& config, uint32_t level);

    /**
     * @brief MPCC: Default constructor, allocate memory
     */
    LMPCC()
    {
        this->reconfigure_server_.reset();      // Reset reconfigure server
    };

    /**
     * @brief ~MPCC: Default distructor, free memory
     */
    ~LMPCC();

    /**
     * @brief initialize: Initialize all helper class of lmpcc control and initialize ros subscribers and publishers
     * @return: Returns true if initialized successfully, false otherwise
     */
    bool initialize();


    bool initialize_visuals();

    /**
     * @brief StateCallBack: Get current state of the robot
     * @param msg: Read data from mobile_robot_state_publisher_node default type:
     */
    void StateCallBack(const geometry_msgs::Pose::ConstPtr& msg);

    /**
     * @brief ObstacleCallBack: Get current state of moving obstacles
     * @param obstacles: Data containing the current moving obstacle configurations
     */
    void ObstacleCallBack(const lmpcc_msgs::lmpcc_obstacle_array& received_obstacles);

    /**
     * @brief getTransform: Find transformation stamed rotation is in the form of quaternion
     * @param from: source frame from find transformation
     * @param to: target frame till find transformation
     * @param stamped_pose: Resultant poseStamed between source and target frame
     * @return: true if transform else false
     */
    bool getTransform(const std::string& from,
                                        const std::string& to,
                                        geometry_msgs::PoseStamped& stamped_pose
                                        );

    /**
     * @brief transformStdVectorToEigenVector: tranform std vector to eigen vectors as std vectos are slow to random access
     * @param vector: std vectors want to tranfrom
     * @return Eigen vectors transform from std vectos
     */
    template<typename T>
    static inline Eigen::VectorXd transformStdVectorToEigenVector(const std::vector<T>& vector)
    {
        // resize eigen vector
        Eigen::VectorXd eigen_vector = Eigen::VectorXd(vector.size());

        // convert std to eigen vector
        for (uint32_t i = 0; i < vector.size(); ++i)
        {
            eigen_vector(i) = vector.at(i);
        }

        return eigen_vector;
    }

    /** public data members */

    ros::ServiceClient map_service_, update_trigger;

    // joint state subsciber to get current joint value
    ros::Subscriber robot_state_sub_;

    // subscriber for obstacle feed
    ros::Subscriber obstacle_feed_sub_;

    // controlled joint velocity, should be control velocity of controller
    ros::Publisher controlled_velocity_pub_;

    // publish trajectory
    ros::Publisher traj_pub_, tr_path_pub_, pred_traj_pub_, pred_cmd_pub_,cost_pub_,robot_collision_space_pub_, global_plan_pub_,local_spline_traj_pub1_, local_spline_traj_pub2_, local_spline_traj_pub3_, local_spline_traj_pub4_, local_spline_traj_pub5_, contour_error_pub_, feedback_pub_, collision_free_pub_;
	//Predicted trajectory
	nav_msgs::Path pred_traj_;
	nav_msgs::Path pred_cmd_;
	nav_msgs::Path local_spline_traj1_,local_spline_traj2_,local_spline_traj3_,local_spline_traj4_,local_spline_traj5_;
	nav_msgs::OccupancyGrid environment_grid_;
    nav_msgs::GetMap map_srv_;

    std_srvs::Empty emptyCall;

	int segment_counter;

	//Controller options
    real_t te_, te_collision_free_;

	tf2_ros::TransformBroadcaster state_pub_, path_pose_pub_;
	std_msgs::Float64 cost_;
    double contour_error_;
    double lag_error_;
    int n_search_points_;

    std::string cmd_topic_;

	//Spline trajectory generation
	tk::spline ref_path_x, ref_path_y;

	//MPCC Implementation
    std::vector<double> ss,xx,yy,vv;
    std::vector<double> obst1_x, obst1_y, obst2_x, obst2_y;

    //Search window parameters
    bool goal_reached_;
    bool last_poly_;
    std::vector<double> collision_free_C1, collision_free_C2, collision_free_C3, collision_free_C4, collision_free_a1x ,collision_free_a1y, collision_free_a2x ,collision_free_a2y, collision_free_a3x ,collision_free_a3y, collision_free_a4x ,collision_free_a4y , collision_free_xmin, collision_free_xmax, collision_free_ymin, collision_free_ymax;

    ReferencePath referencePath;

private:
    ros::NodeHandle nh;

    tf::TransformListener tf_listener_;

    double r_discs_;
    Eigen::VectorXd x_discs_;

    // Timer
    ros::Timer timer_;

    // used to set desired position by mannually or using interactive marker node
    bool tracking_;
    std::string target_frame_;

    visualization_msgs::MarkerArray traj_marker_array_;

    Eigen::Vector3d current_state_, last_state_;
    Eigen::Vector3d goal_pose_, prev_pose_;

	//TRajectory execution variables
	int idx;

	visualization_msgs::Marker ellips1, cube1, global_plan;

    // Obstacles
    lmpcc_msgs::lmpcc_obstacle_array obstacles_;
    lmpcc_msgs::lmpcc_obstacle_array obstacles_init_;

    // Current and last position and velocity from joint state callback
    //Eigen::VectorXd current_position_;
    Eigen::VectorXd last_position_;
    //Eigen::VectorXd current_velocity_;
    Eigen::VectorXd last_velocity_;

    // Type of variable used to publish joint velocity
    geometry_msgs::Twist controlled_velocity_;

    // lmpcc configuration
    boost::shared_ptr<LMPCC_configuration> lmpcc_config_;

    // move to goal position action

	/** Visualization variables **/

    /** Action interface **/
    moveit_msgs::RobotTrajectory traj;      //MoveIt TRAJECTORY VARIABLE

    lmpcc::moveResult move_action_result_;
    lmpcc::moveFeedback move_action_feedback_;
    lmpcc::trajActionFeedback moveit_action_feedback_;
    lmpcc::trajActionResult moveit_action_result_;

    boost::scoped_ptr<actionlib::SimpleActionServer<lmpcc::moveAction> > move_action_server_;

    boost::scoped_ptr<actionlib::SimpleActionServer<lmpcc::trajAction> > moveit_action_server_;

    void moveGoalCB();
    void movePreemptCB();
	void moveitGoalCB();
    void actionSuccess();
    void actionAbort();

    /** Reconfigurable parameters **/
    Eigen::VectorXd min_velocity_limit_,max_velocity_limit_;

    Eigen::VectorXd cost_contour_weight_factors_;
    Eigen::VectorXd cost_control_weight_factors_;

    double slack_weight_;
    double repulsive_weight_;

    double reference_velocity_;
    double collision_free_delta_max_, collision_free_delta_min_;
    int occupied_threshold_;

    bool enable_output_;
    bool loop_mode_;
    int n_iterations_;

    int n_traj_per_cloth;
    double window_size_;

    /**
     * @brief spinNode: spin node means ROS is still running
     */
    void spinNode();

    void computeEgoDiscs();
    /**
     * @brief controlLoop: Continue updating this function depend on controller frequency
     * @param event: Used for computation of duration of first and last event
     */
    void controlLoop(const ros::TimerEvent& event);

    /**
     * @brief publishZeroJointVelocity: published zero joint velocity is statisfied cartesian distance
     */
    void publishZeroJointVelocity();

    void publishTrajectory(void);
	/**
	 * @brief publishPredictedTrajectory: publish predicted trajectory
	 */
	void publishPredictedTrajectory(void);

	void publishGlobalPlan(void);

	void publishLocalRefPath(void);

	void publishPredictedOutput(void);

	void publishPredictedCollisionSpace(void);

	void publishCost(void);

    void publishContourError(void);

//    void publishPathFromTrajectory(const moveit_msgs::RobotTrajectory& traj);

	void broadcastTF();

	void broadcastPathPose();

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
     * @brief getRotatedOccupancy: Returns the occupancy cell value at specified index, for a rotated map
     * @param x_i: Index of grid cell in x direction
     * @param y_i: Index of grid cell in y direction
     * @param psi: Rotation of the map
     */
    int getRotatedOccupancy(int x_i, int search_x, int y_i, int search_y, double psi);

    void ZRotToQuat(geometry_msgs::Pose& pose);

    /**
     * @brief publishPosConstraint: Publish the approximated collision free area as a markerarray
     */
    void publishPosConstraint();

    void publishFeedback(int& it, double& time);

    /**
     * @brief clearDataMember: clear vectors means free allocated memory
     */
    void clearDataMember();
};

#endif // LMPCC_CONTROLLER_H
