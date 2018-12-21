cmake_minimum_required(VERSION 2.8.3)
project(lmpcc_msgs)

find_package(catkin REQUIRED COMPONENTS nav_msgs std_msgs)

include_directories(include)

add_message_files(
  DIRECTORY msg
  FILES lmpcc_feedback.msg lmpcc_obstacle.msg lmpcc_obstacle_array.msg
)

add_service_files(
    DIRECTORY srv
    FILES IntTrigger.srv
)

generate_messages(DEPENDENCIES nav_msgs std_msgs geometry_msgs)

catkin_package(
  DEPENDS nav_msgs std_msgs
  #  INCLUDE_DIRS include
)

install(DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)
