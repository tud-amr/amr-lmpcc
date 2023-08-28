# Model Predictive Contouring Control for Collision Avoidance in Unstructured Dynamic Environments

This repository contains the code for the paper:

**<a href="https://ieeexplore.ieee.org/document/8768044">Model Predictive Contouring Control for Collision Avoidance in Unstructured Dynamic Environments</a>**
<br>
<a href="http://www.tudelft.nl/staff/bruno.debrito/">Bruno Brito</a>,
<a href="">Boaz Floor</a>,
<a href="http://www.tudelft.nl/staff/L.Ferranti/">Laura Ferranti</a>,
<a href="http://www.tudelft.nl/staff/j.alonsomora/">Javier Alonso-Mora</a>
<br>
Accepted in [RA-L + IROS 2019].

This paper proposed a local planning approach based on Model Predictive Contouring Control (MPCC) to safely navigate a mobile robot in dynamic, unstructured environments.
Our local MPCC relies on an upper bound of the Minkowski sum of a circle and an ellipse to safely avoid dynamic obstacles and a set of convex regions in free space to avoid static obstacles.
We compared our design with three baseline approaches (classical MPC, Dynamic Window, and CADRL). The experimental results demonstrate that our method outperforms the baselines in static and dynamic environments. Moreover, the light implementation of our design shows the scalability of our method up to six agents and allowed us to run all algorithms on-board. Finally, we showed the applicability of our design to more complex robots by testing the design in simulation using the model of an autonomous car.
As future work, we intend to expand our approach for crowded scenarios, by accounting for the interaction effects between the robot and the other agents.
Please click in the image to see our video.

<div align='center'>
<a href="https://youtu.be/2ulhqQIXFqQ"><img src="images/paper.png"></img></a>
</div>

If you find this code useful in your research then please cite:
```
@article{britolmpcc,
  title={Model Predictive Contouring Control for Collision Avoidance in Unstructured Dynamic Environments},
  author={Bruno Brito, Boaz Floor, Laura Ferranti and Javier Alonso-Mora},
  year={2019},
  publisher={RA-L - IEEE Robotics and Automation Society}
}
```

The authors would like to thank Wilko Schwarting for sharing its Matlab implementation of the MPCC controller as described in:
```
@article{schwarting2017parallel,
  title={Parallel autonomy in automated vehicles: Safe motion generation with minimal intervention},
  author={Schwarting, Wilko and Alonso-Mora, Javier and Paull, Liam and Karaman, Sertac and Rus, Daniela},
  year={2017},
  publisher={Institute of Electrical and Electronics Engineers (IEEE)}
}
```
## Software Requirements
* ROS installation
* Ubuntu
* Pedestrian Simulator: pedsim_ros: https://github.com/weemma/pedsim_ros
    * Branch: four_persons

## Instalation instructions
This set of instructions were tested for Ubuntu16.04 with ROS-Kinetic and Ubuntu20.04 with ROS-Noetic. Additionally, we assume that you already have a complete ROS installation.
* Please follow the following instructions:
```
sudo apt-get install ros-$ROS_DISTRO-jackal-control ros-$ROS_DISTRO-jackal-gazebo ros-$ROS_DISTRO-jackal-simulator ros-$ROS_DISTRO-jackal-description ros-$ROS_DISTRO-jackal-desktop ros-$ROS_DISTRO-jackal-navigation ros-$ROS_DISTRO-jackal-viz ros-$ROS_DISTRO-people-msgs
cd [catkin_workspace]/src
git clone --recursive https://github.com/weemma/pedsim_ros.git -b four_persons
git clone https://github.com/tud-amr/amr-lmpcc.git
cd ../
rosdep install --from-paths src --ignore-src -r -y
catkin_make
source devel/setup.bash
```

## Solver code
Repository: https://github.com/bbrito/acado/blob/four_persons/examples/ocp/bioreactor.cpp

## Running LMPCC
* Simulation Environment

        1. Start Jackal Gazebo Simulation
            * roslaunch jackal_gazebo jackal_world.launch
        2. Start Pedestrian Simulator
            * roslaunch pedsim_simulator corridor.launch
* Start lmpcc_obstacle_feed

        1. roslaunch lmpcc_obstacle_feed lmpcc_obstacle_feed.launch

* Start lmpcc controller

        1. roslaunch lmpcc lmpcc.launch

* Start rqt_reconfigure

        1. rosrun rqt_reconfigure rqt_reconfigure
        2. Click on the lmpcc parameters to start the robot motion by press the enable_output button
        3. Click on the lmpcc parameters to start planning by pressing the plan button

## Troubleshooting
* If you get the following error:
```
lmpcc_controller.h:146:63: fatal error: static_collision_avoidance/collision_free_polygon.h: No such file or directory
```
Try to compile again using catkin_make several times. Sometimes catkin fails to compile the diferent packages in the right order.
