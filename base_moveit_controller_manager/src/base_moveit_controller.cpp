/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2016, Robert Haschke
 *   Copyright (c) 2017, Bruno Brito
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the names of the authors nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ioan Sucan, Robert Haschke, Bruno Brito */

#include <base_moveit_controller_manager/base_moveit_controller.h>
#include <ros/param.h>
#include <sensor_msgs/JointState.h>
#include <boost/thread.hpp>
#include <limits>

namespace base_moveit_controller_manager
{
BaseControllerHandle::BaseControllerHandle(const std::string& name, const std::vector<std::string>& joints,
                                       const ros::Publisher& pub)
  : moveit_controller_manager::MoveItControllerHandle(name), joints_(joints), pub_(pub)
{
  std::stringstream ss;
  ss << name << "' with joints [ ";
  std::copy(joints.begin(), joints.end(), std::ostream_iterator<std::string>(ss, " "));
  ss << "]";
  ROS_INFO_STREAM(ss.str());
  static const std::string MOVEIT_ACTION_NAME = "fake_base_controller";
  moveit_action_client_.reset(new actionlib::SimpleActionClient<lmpcc::trajAction>("/fake_base_controller", this));
	ROS_INFO("Waiting for MPC Controller");
	moveit_action_client_->waitForServer();
	ROS_INFO("MPC Controller Found!");
}

void BaseControllerHandle::getJoints(std::vector<std::string>& joints) const
{
  joints = joints_;
}

moveit_controller_manager::ExecutionStatus BaseControllerHandle::getLastExecutionStatus()
{
  return moveit_controller_manager::ExecutionStatus::SUCCEEDED;
}

ThreadedController::ThreadedController(const std::string& name, const std::vector<std::string>& joints,
                                       const ros::Publisher& pub)
  : BaseControllerHandle(name, joints, pub)
{
}

ThreadedController::~ThreadedController()
{
  ThreadedController::cancelTrajectory();
}

void ThreadedController::cancelTrajectory()
{
  cancel_ = true;
  thread_.join();
}

bool ThreadedController::sendTrajectory(const moveit_msgs::RobotTrajectory& t)
{
  cancelTrajectory();  // cancel any previous fake motion
  cancel_ = false;
  status_ = moveit_controller_manager::ExecutionStatus::PREEMPTED;
  thread_ = boost::thread(boost::bind(&ThreadedController::execTrajectory, this, t));
  return true;
}

bool ThreadedController::cancelExecution()
{
  cancelTrajectory();
  ROS_INFO("Fake trajectory execution cancelled");
  status_ = moveit_controller_manager::ExecutionStatus::ABORTED;
  return true;
}

bool ThreadedController::waitForExecution(const ros::Duration&)
{
  thread_.join();
  status_ = moveit_controller_manager::ExecutionStatus::SUCCEEDED;
  return true;
}

moveit_controller_manager::ExecutionStatus ThreadedController::getLastExecutionStatus()
{
  return status_;
}

BaseBodyController::BaseBodyController(const std::string& name, const std::vector<std::string>& joints, const ros::Publisher& base_pub)
  : ThreadedController(name, joints, base_pub), rate_(10)
{
  double r;
  if (ros::param::get("~fake_interpolating_controller_rate", r))
    rate_ = ros::WallRate(r);
}

namespace
{
void interpolate(sensor_msgs::JointState& js, const trajectory_msgs::JointTrajectoryPoint& prev,
                 const trajectory_msgs::JointTrajectoryPoint& next, const ros::Duration& elapsed)
{
  double duration = (next.time_from_start - prev.time_from_start).toSec();
  double alpha = 1.0;
  if (duration > std::numeric_limits<double>::epsilon())
    alpha = (elapsed - prev.time_from_start).toSec() / duration;

  js.position.resize(prev.positions.size());
  for (std::size_t i = 0, end = prev.positions.size(); i < end; ++i)
  {
    js.position[i] = prev.positions[i] + alpha * (next.positions[i] - prev.positions[i]);
  }

}
}

std::vector<tk::spline> BaseBodyController::referencePath(const moveit_msgs::RobotTrajectory& trajectory, double& spline_length, const int& n_points_spline, double& dist_spline_pts) {

    int traj_n = trajectory.multi_dof_joint_trajectory.points.size();
    std::vector<double> X, Y, S;
    double x0, y0, x1, y1;
    double s = 0;

    // Store given trajectory x- and y-positions in a vector
    for (int traj_it = 0; traj_it < traj_n - 1; traj_it++)
    {
        x0 = trajectory.multi_dof_joint_trajectory.points[traj_it].transforms[0].translation.x;
        y0 = trajectory.multi_dof_joint_trajectory.points[traj_it].transforms[0].translation.y;

        x1 = trajectory.multi_dof_joint_trajectory.points[traj_it + 1].transforms[0].translation.x;
        y1 = trajectory.multi_dof_joint_trajectory.points[traj_it + 1].transforms[0].translation.y;

        // Keep track of the traveled distance
        s += sqrt( pow(x1 - x0,2) + pow(y1 - y0,2));

        if(traj_it == 0)
        {
            X.push_back(x0);
            Y.push_back(y0);
            S.push_back(0);

            X.push_back(x1);
            Y.push_back(y1);
            S.push_back(s);
        } else{
            X.push_back(x1);
            Y.push_back(y1);
            S.push_back(s);
        }
    }

    // Total length of the trajectory
    spline_length = S.back();

    //Initialize and build splines for x and y
    tk::spline ref_path_x, ref_path_y;
    ref_path_x.set_points(S, X);
    ref_path_y.set_points(S, Y);

    // Distance between spline points
    dist_spline_pts = spline_length / n_points_spline;
    std::vector<double> ss(n_points_spline),xx(n_points_spline),yy(n_points_spline);

    // Sample equally distributed spline points
    for (int i=0; i<n_points_spline; i++){
        ss[i] = dist_spline_pts * i;
        xx[i] = ref_path_x(ss[i]);
        yy[i] = ref_path_y(ss[i]);
    }

    ref_path_x.set_points(ss,xx);
    ref_path_y.set_points(ss,yy);

    // Create vector of the two splines
    std::vector<tk::spline> ref_path_(2);
    ref_path_[0] = ref_path_x;
    ref_path_[1] = ref_path_y;

    ROS_INFO_STREAM("Fitted spline through trajectory points." << std::endl << "Trajectory length: " << spline_length << std::endl << "Number of trajectory points: " << X.size() << std::endl << "Number of spline points: " << n_points_spline );

    return ref_path_;
}

// A parameter vector is created of all spline parameters, similar to the MATLAB implementation
void BaseBodyController::insertParams(moveit_msgs::RobotTrajectory& trajectory,const std::vector<tk::spline>& spline, const double& spline_length, const int& n_points_spline, const double& dist_spline_pts)
{

//    int length_p = 20000;   // total parameter vector length (n * number of parameters)
    int length_p = 1500;   // total parameter vector length (n * number of parameters)
    int N = 12;             // prediction horizon length
    int spline_index = 500; // Offset in parameter vector when spline parameters start
    int k;                  // Index in parameter vector for time horizon steps

    // We are going to store the whole vector in the RobotTrajectory structure
    trajectory.joint_trajectory.points.resize(1);
    trajectory.joint_trajectory.points[0].positions.resize(length_p);

    // Iterate over all horizon steps to insert parameters
//    for (int N_it = 0; N_it < N; N_it++)
//    {

//      k = N_it*N;
      k = 0;
      trajectory.joint_trajectory.points[0].positions[k + 1] = n_points_spline;
      trajectory.joint_trajectory.points[0].positions[k + 2] = dist_spline_pts;

      // Insert spline coefficients of the separate x- and y-coordinate splines
      for (int j = 0 ; j<(n_points_spline - 1); j++ ) {
        trajectory.joint_trajectory.points[0].positions[k + spline_index + j * 8] = spline[0].m_a[n_points_spline];
        trajectory.joint_trajectory.points[0].positions[k + spline_index + j * 8 + 1] = spline[0].m_b[n_points_spline];
        trajectory.joint_trajectory.points[0].positions[k + spline_index + j * 8 + 2] = spline[0].m_c[n_points_spline];
        trajectory.joint_trajectory.points[0].positions[k + spline_index + j * 8 + 3] = spline[0].m_d[n_points_spline];
        trajectory.joint_trajectory.points[0].positions[k + spline_index + j * 8 + 4] = spline[1].m_a[n_points_spline];
        trajectory.joint_trajectory.points[0].positions[k + spline_index + j * 8 + 5] = spline[1].m_b[n_points_spline];
        trajectory.joint_trajectory.points[0].positions[k + spline_index + j * 8 + 6] = spline[1].m_c[n_points_spline];
        trajectory.joint_trajectory.points[0].positions[k + spline_index + j * 8 + 7] = spline[1].m_d[n_points_spline];
        }
//    }

     ROS_INFO_STREAM("Insterted parameters in paramater vector");

}

	void BaseBodyController::transformPose(const std::string source_frame, const std::string target_frame, const geometry_msgs::Pose pose_in, geometry_msgs::Pose& pose_out)
	{
		ROS_WARN_STREAM("MPCC::transformPose ... find transformation matrix between "
							<< target_frame.c_str() << " and " << source_frame.c_str());
		bool transform = false;
		geometry_msgs::PoseStamped stamped_in, stamped_out;
		stamped_in.header.frame_id = source_frame;
		stamped_in.pose = pose_in;
		do
		{
			try
			{
				if (tf_listener_.frameExists(target_frame))
				{
					// clear output
					pose_out = geometry_msgs::Pose();

					ros::Time now = ros::Time::now();
					tf_listener_.waitForTransform(target_frame, source_frame, now, ros::Duration(0.1));
					tf_listener_.transformPose(target_frame, stamped_in, stamped_out);
					pose_out = stamped_out.pose;
					transform = true;
				}

				else
				{
					ROS_WARN_STREAM("MPCC::transformPose" << target_frame.c_str() << " does not exist");
					transform = false;
				}
			}
			catch (tf::TransformException& ex)
			{
				ROS_ERROR("MPCC::transformPose: \n%s", ex.what());
				ros::Duration(0.1).sleep();
			}
		} while (!transform && ros::ok());
	}

void BaseBodyController::execTrajectory(const moveit_msgs::RobotTrajectory& t)
{
	ROS_WARN("WHOLE BODY CONTROLLER execution of trajectory");
	lmpcc::trajGoal goal, spline_goal;
	goal.trajectory.multi_dof_joint_trajectory = t.multi_dof_joint_trajectory;

    double ysqr, t3, t4;
  
	ROS_WARN_STREAM("t: " << t);
	geometry_msgs::Pose pose_in,pose_out;
	// convert traj to base_link
	/*
	for ( int i = 0; i< ((int)goal.trajectory.multi_dof_joint_trajectory.points.size()) ; i++ )
	{
		pose_in.position.x = goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].translation.x;
		pose_in.position.y = goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].translation.y;
		pose_in.position.z = 0;
		pose_in.orientation.x = goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.x;
		pose_in.orientation.y = goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.y;
		pose_in.orientation.z = goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z;
		pose_in.orientation.w = goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.w;
		transformPose("odom", "base_link", pose_in, pose_out);

		goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].translation.x=pose_out.position.x;
		goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].translation.y=pose_out.position.y;
		goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].translation.z=pose_out.position.z;
		goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.x=pose_out.orientation.x;
		goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.y=pose_out.orientation.y;
		goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z=pose_out.orientation.z;
		goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.w=pose_out.orientation.w;
	}

    // Convert quaternions to euler angle and store in rotation.z
    for ( int i = 0; i< ((int)goal.trajectory.multi_dof_joint_trajectory.points.size()) ; i++ )
    {
        ysqr = goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.y * goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.y;
        t3 = +2.0 * (goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.w * goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z
                      + goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.x * goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.y);
        t4 = +1.0 - 2.0 * (ysqr + goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z * goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z);
  
        goal.trajectory.multi_dof_joint_trajectory.points[i].transforms[0].rotation.z = atan2(t3, t4);
    }
*/
    // Initialize x- and y-coordinate spline
    std::vector<tk::spline> ref_path(2);
    double spline_length;
    int n_points_spline = 30;   // Number of points the spline should be resampled with
    double dist_spline_points;  // Initialize distance between spline points

    // Compute local reference path as a spline
    ref_path = referencePath(goal.trajectory,spline_length,n_points_spline,dist_spline_points);

    // Construct new goal structure to store parameters in
    spline_goal = goal;
    insertParams(spline_goal.trajectory,ref_path,spline_length,n_points_spline,dist_spline_points);

    // Initialize moveit action
	moveit_action_client_->sendGoal(spline_goal);

}

}  // end namespace base_moveit_controller_manager
