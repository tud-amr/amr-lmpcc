#! /usr/bin/env python

import rospy
import sys
# Brings in the SimpleActionClient
import actionlib
import math
import tf2_ros
from geometry_msgs.msg import *
from std_srvs.srv import *
import time
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
#x= [50, 55, 60, 65,70,80]
#y= [-0.5, -0.5, -0.5, -0.5,-0.5,-0.5]
#theta= [0,0,0,0,0,0]
#reference_velocity= 0.5
#cadrl test
x= [15]
y= [0]
theta= [0]
reference_velocity= 0.5
distance_threshold = 0.63

loop = True

def cadrl_client(index,pub_global_goal):
    # Creates a goal to send to the action server.
    goal = PoseStamped()
    goal.header.stamp = rospy.get_rostime()
    goal.header.frame_id = "odom"
    goal.pose.position.x = x[index]
    goal.pose.position.y = y[index]
    goal.pose.orientation.x = 0
    goal.pose.orientation.y = 0
    goal.pose.orientation.z = math.sin(theta[i]*0.5)
    goal.pose.orientation.w = math.cos(theta[i]*0.5)
    # Sends the goal to the action server.
    pub_global_goal.publish(goal)

def check_if_arrived(i,tfBuffer):

  try:
    trans = tfBuffer.lookup_transform('odom', 'base_link', rospy.Time(0))
  except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
    print ("Could not get TF")
    return False

  if math.sqrt(pow(x[i]-trans.transform.translation.x,2)+pow(y[i]-trans.transform.translation.y,2)) < 1:
    return True
  else:
    return False

def collision_check(tfBuffer):
    try:
        pos_ped_1 = tfBuffer.lookup_transform('base_link', 'ped_link_1', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get ped_link_1" )
        return False
    try:
        pos_ped_2 = tfBuffer.lookup_transform('base_link', 'ped_link_2', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get ped_link_2" )
        return False
    try:
        pos_ped_3 = tfBuffer.lookup_transform('base_link', 'ped_link_3', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get ped_link_3" )
        return False
    try:
        pos_ped_4 = tfBuffer.lookup_transform('base_link', 'ped_link_4', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get ped_link_4" )
        return False
    """
    try:
        pos_ped_5 = tfBuffer.lookup_transform('base_link', 'ped_link_5', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get ped_link_5" )
        return False
    try:
        pos_ped_6 = tfBuffer.lookup_transform('base_link', 'ped_link_6', rospy.Time(0))
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        print ("Could not get ped_link_6" )
        return False
    """
    ped_distance_1 =math.sqrt(pow(pos_ped_1.transform.translation.x,2)+pow(pos_ped_1.transform.translation.y,2))
    #print("ped_distance_1: " +str(ped_distance_1))
    if  ped_distance_1 < distance_threshold:
        print ("Collision with ped_link_1!!!")
        return True
    #print("ped_distance_1: " +str(ped_distance_1))
    if math.sqrt(pow(pos_ped_2.transform.translation.x,2)+pow(pos_ped_2.transform.translation.y,2)) < distance_threshold:
        print ("Collision with ped_link_2")
        return True
    if math.sqrt(pow(pos_ped_3.transform.translation.x,2)+pow(pos_ped_3.transform.translation.y,2)) < distance_threshold:
        print ("Collision with ped_link_3")
        return True
    if math.sqrt(pow(pos_ped_4.transform.translation.x,2)+pow(pos_ped_4.transform.translation.y,2)) < distance_threshold:
        print ("Collision with ped_link_4")
        return True
    else:
        return False
    """
    if math.sqrt(pow(pos_ped_5.transform.translation.x,2)+pow(pos_ped_5.transform.translation.y,2)) < distance_threshold:
        print ("Collision with ped_link_5")
        return True
    if math.sqrt(pow(pos_ped_6.transform.translation.x,2)+pow(pos_ped_6.transform.translation.y,2)) < distance_threshold:
        print ("Collision with ped_link_6")
        return True
    """

if __name__ == '__main__':
    rospy.init_node('cadrl_base_client_py')
    i = 0

    pub_global_goal = rospy.Publisher('/nn_jackal/goal',PoseStamped, queue_size=1)
    reset_simulation_client_ = rospy.ServiceProxy("/gazebo/reset_world",Empty());

    """ROS Variables"""
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    collision_number = 0
    n_events = 0
    trials = 0
    timeout = 0
    mean_time=0
    while(i < len(x)):

      if trials > 100:
        break
      try:
        # Initializes a rospy node so that the SimpleActionClient can
        # publish and subscribe over ROS.

        cadrl_client(i,pub_global_goal)
        arrived = False
        col = False
        ti = time.time()           # initial time

        while (not arrived) and (not col):
          #rospy.sleep(0.1)
          cadrl_client(i,pub_global_goal)
          arrived = check_if_arrived(i,tfBuffer)
          if arrived:
              break
          col = collision_check(tfBuffer)
          if col:
              collision_number += 1
              trials +=1
              i=0
              reset_simulation_client_()
              rospy.sleep(1)
              break
          tf = time.time()
          if tf-ti > 90:
              reset_simulation_client_()
              i=0
              timeout += 1
              trials +=1
              break

          #print("Not arrived in: " + str(tf-ti) + " [s]")
      except rospy.ROSInterruptException:
        print("Failed")
        break
      print("next goal pos..."+str(i+1))
      i += 1

      if i == len(x):
          i = 0
          n_events += 1
          trials +=1
          mean_time +=tf-ti
          print("Mean time to goal: " + str(mean_time))
          print("Number of collisions: " + str(collision_number))
          print("Number of successful events: " + str(n_events))
          print("Number of trials: " + str(trials))
          print("Number of timeout: " + str(timeout))
          reset_simulation_client_()
          rospy.sleep(1)

          if trials > 100:
              break
