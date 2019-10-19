cmake_minimum_required(VERSION 2.8.3)
project(lmpcc)

add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
    actionlib
    actionlib_msgs
    cmake_modules
    control_msgs
    dynamic_reconfigure
    eigen_conversions
    geometry_msgs
    nav_msgs
    map_msgs
    roscpp
    roslint
    static_collision_avoidance
    sensor_msgs
    std_msgs
    std_srvs
    tf
    tf_conversions
    trajectory_msgs
    urdf
    visualization_msgs
    shape_msgs
    lmpcc_msgs
)

find_package(Boost REQUIRED COMPONENTS thread)
find_package(Eigen3 REQUIRED)

add_definitions(${EIGEN_DEFINITIONS})

catkin_python_setup()

generate_dynamic_reconfigure_options(
        cfg/Lmpcc.cfg
)

# Generate added messages and services with any dependencies listed here
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
    lmpcc_msgs
)

catkin_package(
  CATKIN_DEPENDS actionlib_msgs dynamic_reconfigure eigen_conversions geometry_msgs nav_msgs roscpp sensor_msgs std_msgs tf tf_conversions urdf visualization_msgs shape_msgs  lmpcc_msgs static_collision_avoidance
  DEPENDS Boost
  INCLUDE_DIRS include/${PROJECT_NAME}
  LIBRARIES lmpcc_configuration lmpcc
)

### BUILD ###
include_directories(include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN_INCLUDE_DIRS}
    include/${PROJECT_NAME}
    include/${PROJECT_NAME}/ocp
    qpoases/include
    qpoases/src
    )

add_library(lmpcc_configuration src/lmpcc_configuration.cpp)
add_dependencies(lmpcc_configuration ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(lmpcc_configuration
    ${catkin_LIBRARIES}
    ${CASADI_LIBRARIES}
    )

add_library(lmpcc
        src/lmpcc_controller.cpp
        src/lmpcc_configuration.cpp
        src/Clothoid.cpp
        src/reference_path.cpp
        qpoases/SRC/Bounds.cpp
        qpoases/SRC/Constraints.cpp
        qpoases/SRC/CyclingManager.cpp
        qpoases/SRC/Indexlist.cpp
        qpoases/SRC/MessageHandling.cpp
        qpoases/SRC/QProblem.cpp
        qpoases/SRC/QProblemB.cpp
        qpoases/SRC/SubjectTo.cpp
        qpoases/SRC/Utils.cpp
        qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
        src/ocp/acado_qpoases_interface.cpp
        src/ocp/acado_hessian_regularization.c
        src/ocp/acado_integrator.c
        src/ocp/acado_solver.c
        src/ocp/acado_auxiliary_functions.c)

target_include_directories(lmpcc PUBLIC
        qpoases
        qpoases/INCLUDE
        qpoases/SRC)

add_library(tkspline src/spline.cpp)
target_link_libraries(tkspline
        ${catkin_LIBRARIES}
        )

add_executable(lmpcc_node src/lmpcc_node.cpp)
target_link_libraries(lmpcc_node
        lmpcc
        ${catkin_LIBRARIES}
        tkspline)

add_dependencies(lmpcc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

### Test Case ####
install(
  DIRECTORY include/lmpcc/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(
  TARGETS
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
