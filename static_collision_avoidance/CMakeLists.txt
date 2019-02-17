cmake_minimum_required(VERSION 2.8.3)
project(static_collision_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        actionlib
        actionlib_msgs
        cmake_modules
        control_msgs
        dynamic_reconfigure
        eigen_conversions
        geometry_msgs
        moveit_core
        nav_msgs
        map_msgs
        roscpp
        roslint
        sensor_msgs
        std_msgs
        std_srvs
        tf
        tf_conversions
        trajectory_msgs
        urdf
        visualization_msgs
        shape_msgs
        moveit_msgs
        lmpcc_msgs
        spencer_tracking_msgs
        )

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES collision_free_polygon.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        actionlib_msgs
        geometry_msgs
        sensor_msgs
        std_msgs
        nav_msgs
        map_msgs
        shape_msgs
        visualization_msgs
        moveit_msgs
        lmpcc_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
        CATKIN_DEPENDS actionlib_msgs dynamic_reconfigure eigen_conversions geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf tf_conversions urdf visualization_msgs shape_msgs moveit_core lmpcc_msgs
        DEPENDS Boost
        INCLUDE_DIRS include/${PROJECT_NAME}
        LIBRARIES
)

### BUILD ###
include_directories(include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        include/${PROJECT_NAME}
        )

add_executable(static_collision_avoidance_node src/static_collision_avoidance_node.cpp src/static_environment.cpp)
target_link_libraries(static_collision_avoidance_node
        ${catkin_LIBRARIES}
        )

add_dependencies(static_collision_avoidance_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

