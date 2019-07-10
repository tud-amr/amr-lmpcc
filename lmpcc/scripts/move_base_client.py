#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import math
import tf2_ros
import geometry_msgs.msg
from std_srvs.srv import *
import time
# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.

import move_base_msgs.msg as move

"""Reference Path"""
#x = [1.5, 3.5, 5.5, 7, 5.5, 3.5, 1.5, 0, 1.5]
#y = [0.5, 0.5, 0.5, 2, 3.5, 3.5, 3.5, 2, 0.5]
#theta = [0, 0, 0, 1.57, 3.14, 3.14, 3.14, -1.57, 0]

#global_path:
#x = [1.5, 3.5, 5.5, 7, 8, 10, 13, 11, 10.5, 9, 7, 5.5, 3.5, 1.5, 0, 1.5]
#y = [0.5, 0.5, 0.5, 2, 4, 4, 6, 7.5, 6, 4, 3.5, 3.5, 3.5, 3.5, 2, 0.5]
#theta = [0, 0, 0, 1.57, 0, 0, 1.57, 3.14, -1.57, 3.14, 3.14,  3.14, 3.14, 3.14, -1.57, 0]

# faculty corridor
#global_path:
x= [15]
y= [0]
theta= [0]
reference_velocity= 0.5
distance_threshold = 0.63

loop = True

def move_base_client(index):
    # Creates the SimpleActionClient, passing the type of the action
    # (FibonacciAction) to the constructor.
    client = actionlib.SimpleActionClient('move_base', move.MoveBaseAction)

    # Waits until the action server has started up and started
    print("Waiting for server")
    success = client.wait_for_server(rospy.rostime.Duration(10))
    if success:
        # Creates a goal to send to the action server.
        goal = move.MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x[index]
        goal.target_pose.pose.position.y = y[index]
        goal.target_pose.pose.orientation.x = 0
        goal.target_pose.pose.orientation.y = 0
        goal.target_pose.pose.orientation.z = math.sin(theta[i]*0.5)
        goal.target_pose.pose.orientation.w = math.cos(theta[i]*0.5)
        # Sends the goal to the action server.
        client.send_goal(goal)
        return True
    else:
        return False

    # Waits for the server to finish performing the action.
    #client.wait_for_result()

def check_if_arrived(i,listener):

  try:
    trans = listener.lookup_transform('map', 'base_link', rospy.Time(0))
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print ("Could not get TF")
    return False

  if math.sqrt(pow(x[i]-trans.transform.translation.x,2)+pow(y[i]-trans.transform.translation.y,2)) < 0.75:
    print ("Arrived!!!")
    return True
  else:
    return False

def collision_check(listener):
    try:
        pos_ped_1 = listener.lookup_transform('base_link', 'ped_link_1', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get TF")
        return False
    try:
        pos_ped_2 = listener.lookup_transform('base_link', 'ped_link_2', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get TF")
        return False
    try:
        pos_ped_3 = listener.lookup_transform('base_link', 'ped_link_3', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get TF")
        return False
    try:
        pos_ped_4 = listener.lookup_transform('base_link', 'ped_link_4', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get TF")
        return False
    """
    try:
        pos_ped_5 = listener.lookup_transform('base_link', 'ped_link_5', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get TF")
        return False
    try:
        pos_ped_6 = listener.lookup_transform('base_link', 'ped_link_6', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get TF")
        return False
    """
    ped_distance_1 =math.sqrt(pow(pos_ped_1.transform.translation.x,2)+pow(pos_ped_1.transform.translation.y,2))
    #print("ped_distance_1: " +str(ped_distance_1))
    if  ped_distance_1 < distance_threshold:
        print ("Collision with ped_link_1")
        return True
    #print("ped_distance_1: " +str(ped_distance_1))
    if math.sqrt(pow(pos_ped_2.transform.translation.x,2)+pow(pos_ped_2.transform.translation.y,2)) < distance_threshold:
        print ("Collision with ped_link_2")
        return True
    if math.sqrt(pow(pos_ped_3.transform.translation.x,2)+pow(pos_ped_3.transform.translation.y,2)) < distance_threshold:
        print ("Collision with ped_link_3")
        return True
        return False
    if math.sqrt(pow(pos_ped_4.transform.translation.x,2)+pow(pos_ped_4.transform.translation.y,2)) < distance_threshold:
        print ("Collision with ped_link_4")
        return True
    else:
        return False
    """
    if math.sqrt(pow(pos_ped_5.transform.translation.x,2)+pow(pos_ped_5.transform.translation.y,2)) < distance_threshold:
        print ("Collision with ped_link_5")
        return True
        return False
    if math.sqrt(pow(pos_ped_6.transform.translation.x,2)+pow(pos_ped_6.transform.translation.y,2)) < distance_threshold:
        print ("Collision with ped_link_6")
        return True
    """



if __name__ == '__main__':

    # Initializes a rospy node so that the SimpleActionClient can
    # publish and subscribe over ROS.
    rospy.init_node('move_base_client_py')
    print("ROS Variables")
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    i = 0
    collision_number = 0
    n_events = 0
    trials = 0
    reset_simulation_client_ = rospy.ServiceProxy("/gazebo/reset_world",Empty())
    clean_costmap_client_ = rospy.ServiceProxy("/move_base/clear_costmaps", Empty());
    timeout = 0
    n_trials = 0
    while(i < len(x)):
      if trials > 100:
        break
      try:

        moving = move_base_client(i)
        if moving:
          arrived = False
          col = False
          ti = time.time()           # initial time

          while (not arrived) and (not col):
            arrived = check_if_arrived(i,tfBuffer)
            if arrived:
              break
            col = collision_check(tfBuffer)
            if col:
              print("Collision")
              collision_number += 1
              i=0
              trials +=1
              reset_simulation_client_()
              clean_costmap_client_()
              rospy.sleep(1)
              break
            tf = time.time()
            if tf-ti > 90:
              print("Timeout")
              reset_simulation_client_()
              clean_costmap_client_()
              timeout += 1
              trials +=1
              break
        else:
            print("Call again server...")
      except rospy.ROSInterruptException:
        print("Failed")
        break
      if col:
        i=0
      else:
        i += 1
      if i == len(x):
        trials +=1
        i = 0
        n_events += 1
        print("Number of collisions: " + str(collision_number))
        print("Number of timeout: " + str(timeout))
        print("Number of successful events: " + str(n_events))
        print("Number of events: " + str(trials))
        reset_simulation_client_()
        clean_costmap_client_()

        if trials > 100:
            break

        rospy.sleep(1)
