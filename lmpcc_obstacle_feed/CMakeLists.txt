cmake_minimum_required(VERSION 2.8.3)
project(lmpcc_obstacle_feed)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  people_msgs
  nav_msgs
  roscpp
  sensor_msgs
  shape_msgs
  std_msgs
  std_srvs
  visualization_msgs
  tf
  trajectory_msgs
  vision_msgs
  lmpcc_msgs
  spencer_tracking_msgs
  pedsim_msgs
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})


generate_dynamic_reconfigure_options(
        cfg/ObstacleFeed.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS actionlib actionlib_msgs cmake_modules dynamic_reconfigure geometry_msgs pedsim_msgs  nav_msgs roscpp sensor_msgs shape_msgs std_msgs std_srvs tf trajectory_msgs vision_msgs visualization_msgs lmpcc_msgs
  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
)

add_library(obstacle_prediction src/obstacle_prediction.cpp)
add_dependencies(obstacle_prediction ${obstacle_prediction_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(obstacle_prediction
        ${catkin_LIBRARIES}
        )

add_library(${PROJECT_NAME} src/${PROJECT_NAME}.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

add_library(${PROJECT_NAME}_configuration src/${PROJECT_NAME}_configuration.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
   ${catkin_LIBRARIES}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/lmpcc_obstacle_feed_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
   ${PROJECT_NAME}
   ${PROJECT_NAME}_configuration
   obstacle_prediction
   ${catkin_LIBRARIES}
)