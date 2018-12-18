cmake_minimum_required(VERSION 2.8.3)
project(base_moveit_controller_manager)

add_compile_options(-std=c++11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(Boost REQUIRED thread)
include_directories(${Boost_INCLUDE_DIRS} ${CATKIN_GLOBAL_INCLUDE_DESTINATION})
link_directories(${Boost_LIBRARY_DIRS})

find_package(catkin COMPONENTS
  actionlib
  actionlib_msgs
  control_msgs
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_fake_controller_manager
  lmpcc
  pluginlib
  roscpp
  sensor_msgs
  std_msgs
  trajectory_msgs
		tf
  REQUIRED)

#find_package(cornucopia REQUIRED)

include_directories(include/${PROJECT_NAME} ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  	include/${PROJECT_NAME}
  	${CATKIN_GLOBAL_INCLUDE_DESTINATION}
  	${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  DEPENDS 
    moveit_core
    moveit_ros_planning
    moveit_fake_controller_manager
	lmpcc
    )

include_directories(
	include/${PROJECT_NAME}
	include)


add_library(tkspline_lib
	src/spline.cpp
	)
add_dependencies(tkspline_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(tkspline_lib ${CATKIN_LIBRARIES})

add_library(${PROJECT_NAME}
	src/base_moveit_controller_manager.cpp
	src/base_moveit_controller.cpp)
set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${${PROJECT_NAME}_VERSION})
target_link_libraries(
	${PROJECT_NAME}
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
	tkspline_lib)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES base_moveit_controller_plugin_description.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
       )
