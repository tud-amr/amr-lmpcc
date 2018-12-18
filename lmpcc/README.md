This repository contains a MPC regulator for Jackal.

# predictive_trajectory_generator.cpp
- Contains code to pregenerate MPC solver using ACADO

# predictive_configuration.cpp
- Contains code to MPC load parameters from predictive_config_parameter.yaml in the config directory

# mpcc_controller.cpp
- Contains the controller class that is launched by the predictive_control_node
