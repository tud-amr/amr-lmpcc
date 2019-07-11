cmake_minimum_required(VERSION 2.8.3)
project(mobile_robot_state_publisher)

find_package(catkin REQUIRED COMPONENTS
        geometry_msgs
        roscpp
        rospy
        std_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs
        )

###################################
## catkin specific configuration ##
###################################
catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES turtle_tf2
        CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf2 tf2_ros
        #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        ${catkin_INCLUDE_DIRS}
)

## C++ examples
add_executable(mobile_robot_state_publisher_node src/mobile_robot_state_publisher_node.cpp)
target_link_libraries(mobile_robot_state_publisher_node ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
    add_dependencies(mobile_robot_state_publisher_node ${catkin_EXPORTED_TARGETS})
endif()

## Install C++ Examples
install(TARGETS mobile_robot_state_publisher_node
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )