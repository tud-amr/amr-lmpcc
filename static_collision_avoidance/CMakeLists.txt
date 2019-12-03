cmake_minimum_required(VERSION 2.8.3)
project(static_collision_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        dynamic_reconfigure
        geometry_msgs
        nav_msgs
        map_msgs
        roscpp
        sensor_msgs
        std_msgs
        visualization_msgs
        message_generation
        )

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY msg
   FILES collision_free_polygon.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS message_runtime dynamic_reconfigure geometry_msgs nav_msgs roscpp sensor_msgs std_msgs visualization_msgs 
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

