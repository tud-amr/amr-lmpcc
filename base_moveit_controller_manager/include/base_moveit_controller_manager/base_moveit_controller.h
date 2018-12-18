/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Ioan A. Sucan
 *  Copyright (c) 2016, Robert Haschke
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

#include <moveit/macros/class_forward.h>
#include <moveit/controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <ros/rate.h>
#include <boost/thread/thread.hpp>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <lmpcc/trajAction.h>
#include <lmpcc/trajActionGoal.h>
#include <vector>
#include <tkspline/spline.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

#ifndef BASE_MOVEIT_CONTROLLER_MANAGER
#define BASE_MOVEIT_CONTROLLER_MANAGER


namespace base_moveit_controller_manager
{
MOVEIT_CLASS_FORWARD(BaseControllerHandle);

class BaseControllerHandle : public moveit_controller_manager::MoveItControllerHandle
{
public:
  BaseControllerHandle(const std::string& name, const std::vector<std::string>& joints, const ros::Publisher& pub);

  virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus();
  void getJoints(std::vector<std::string>& joints) const;

protected:
  std::vector<std::string> joints_;
  const ros::Publisher& pub_;
  boost::scoped_ptr<actionlib::SimpleActionClient<lmpcc::trajAction> > moveit_action_client_;
};

class ThreadedController : public BaseControllerHandle
{
public:
  ThreadedController(const std::string& name, const std::vector<std::string>& joints, const ros::Publisher& pub);
  ~ThreadedController();

  virtual bool sendTrajectory(const moveit_msgs::RobotTrajectory& t);
  virtual bool cancelExecution();
  virtual bool waitForExecution(const ros::Duration&);
  virtual moveit_controller_manager::ExecutionStatus getLastExecutionStatus();

protected:
  bool cancelled()
  {
    return cancel_;
  }

private:
  virtual void execTrajectory(const moveit_msgs::RobotTrajectory& t) = 0;
  virtual void cancelTrajectory();

private:
  boost::thread thread_;
  bool cancel_;
  moveit_controller_manager::ExecutionStatus status_;
};

class BaseBodyController : public ThreadedController
{
public:
    BaseBodyController(const std::string& name, const std::vector<std::string>& joints, const ros::Publisher& base_pub);
  ~BaseBodyController(){};

protected:

  std::vector<tk::spline> referencePath(const moveit_msgs::RobotTrajectory& trajectory, double& spline_length, const int& n_points_spline, double& dist_spline_points);
  virtual void insertParams(moveit_msgs::RobotTrajectory& trajectory,const std::vector<tk::spline>& spline,const double& spline_length, const int& n_points_spline, const double& dist_spline_pts);
  virtual void execTrajectory(const moveit_msgs::RobotTrajectory& t);
  virtual void transformPose(const std::string source_frame, const std::string target_frame, const geometry_msgs::Pose pose_in, geometry_msgs::Pose& pose_out);

private:
  ros::WallRate rate_;
    tf::TransformListener tf_listener_;
};

}; /* End of base_moveit_controller_manager namespace */

#endif
