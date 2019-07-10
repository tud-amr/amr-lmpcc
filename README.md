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

# Instalation instructions
