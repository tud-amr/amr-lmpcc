# Local Model Predictive Contouring Control for Dynamic Environments

This repository contains the code for the paper:

**<a href="https://arxiv.org/abs/1803.10892">Local Model Predictive Contouring Control for Dynamic Environments</a>**
<br>
<a href="http://www.tudelft.nl/staff/bruno.debrito/">Bruno Brito</a>,
<a href="">Boaz Floor</a>,
<a href="http://www.tudelft.nl/staff/L.Ferranti/">Laura Ferranti</a>,
<a href="http://www.tudelft.nl/staff/j.alonsomora/">Javier Alonso-Mora</a>
<br>
Accepted in [RA-L + IROS 2019].

This paper presents a method for local motion planning in unstructured environments with static and moving obstacles, such as humans. Given a reference path and speed, our optimization-based receding-horizon approach computes a local trajectory that minimizes the tracking error while avoiding obstacles. We build on nonlinear model-predictive
contouring control (MPCC) and extend it to incorporate a static map by computing, online, a set of convex regions in free space. We model moving obstacles as ellipsoids and provide a correct bound to approximate the collision region, given by the Minkowsky sum of an ellipse and a circle. 
Our framework is agnostic to the robot model. We present experimental results with a mobile robot navigating in indoor environments populated with humans. Our method is executed fully onboard without the need of external support and can be applied to other robot morphologies such as autonomous cars.

<div align='center'>
<img src="images/paper.png"></img>
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
* Pedestrian Simulator: pedsim_ros: https://github.com/bbrito/pedsim_ros
    * Branch: four_persons

## Instalation instructions
This set of instructions were only tested for Ubuntu16 with ROS-Kinetic. Additionally, we assume that you already have a complete ROS installation.
* Install Jackal Software Packages
    sudo apt-get install ros-kinetic-jackal-control ros-kinetic-jackal-gazebo ros-kinetic-jackal-simulator ros-kinetic-jackal-description ros-kinetic-jackal-desktop ros-kinetic-jackal-navigation ros-kinetic-jackal-viz
* Install Pedestrian Simulator
    Follow instruction from the website: https://github.com/bbrito/pedsim_ros


## Running LMPCC
* Simulation Environment

        1. Start Jackal Gazebo Simulation
            * roslaunch jackal_gazebo jackal_world.launch
        2. Start Pedestrian Simulator
            * roslaunch pedsim_simulator corridor.launch
* Start lmpcc_obstacle_feed

        1. roslaunch lmpcc_obstacle_feed lmpcc_obstacle_feed.launch

*Start lmpcc controller

        1. roslaunch lmpcc lmpcc.launch

* Start rqt_reconfigure

        1. rosrun rqt_reconfigure rqt_reconfigure
        2. Click on the lmpcc parameters to start planning by pressing the plan button
        3. Click on the ledsim_simulator_parameters to start the simulation
        4. Click on the lmpcc parameters to start the robot motion by pressin the enable_output button