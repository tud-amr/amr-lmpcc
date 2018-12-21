#ifndef LMPCC_CONFIGURATION_H
#define LMPCC_CONFIGURATION_H

// ros includes
#include <ros/ros.h>

//c++ includes
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <iomanip>
#include <math.h>

class LMPCC_configuration
{
    /**
    *  @brief All neccessary configuration parameter of predictive control repository
    *         Read data from parameter server
    *         Updated old data with new data
    *  Note:  All data member name used like xyz_ and all parameter name is normal like xyz.
    */

public:

    /** function members of class **/

    // constructor and destructor
    /**
    * @brief predictive_configuration: defualt constructor of this class
    */
    LMPCC_configuration();

    /**
    * @brief ~predictive_configuration: defualt destructor of this class
    */
    ~LMPCC_configuration();

    /**
    * @brief intialize:  check parameters on parameter server and read them
    * @return true if all parameters initialize successfully, else false
    */
    bool initialize();

    /**
    * @brief updateConfiguration: update configuration parameter with new parameter
    * @param new_config: changed configuration parameter
    * @return true if all parameter update successfully, false otherwise
    */
    bool updateConfiguration(const LMPCC_configuration& new_config);

    /** data members of class **/

    bool initialize_success_;

    /** Simulation mode **/
    bool simulation_mode_;
    bool gazebo_simulation_;

    /** Debug modes **/
    bool activate_output_;
    bool activate_debug_output_;
    bool activate_timing_output_;
    bool activate_visualization_;
    bool activate_feedback_message_;

    /** controller frequency in Hz **/
    double controller_frequency_;

    /** dimensions of controlled vehicle **/
    int n_discs_;
    double ego_l_;
    double ego_w_;

    /** OCP dimensions **/
    int state_dim_;
    int control_dim_;

    /** publish and subscribe topic definitions **/
    std::string cmd_;
    std::string cmd_sim_;
    std::string robot_state_;
    std::string ellipse_objects_feed_;

    /** coordinate frame declarations **/
    std::string robot_base_link_;
    std::string planning_frame_;

    /** path parametrization **/
    int n_local_;                   // Number of segments in the local reference path
    int n_poly_per_clothoid_;       // Number of polynomiols fitted per clothoid
    int n_search_points_;
    double search_window_size_;

    /** predefined global reference path **/
    std::vector<double> ref_x_;
    std::vector<double> ref_y_;
    std::vector<double> ref_theta_;
    double reference_velocity_;

    /** Number of dynamic obstacles **/
    int n_obstacles_;
    double delta_max_;
    bool free_space_assumption_;
    int occupied_threshold_;

    /** OCP weight factors **/
    std::vector<double> contour_weight_factors_;
    std::vector<double> control_weight_factors_;
    double repulsive_weight_;
    double slack_weight_;

    /** OCP constraint values **/
    std::vector<double> vel_min_limit_;
    std::vector<double> vel_max_limit_;

    /** ACADO configuration **/
    int max_num_iteration_;
    double kkt_tolerance_;
    double integrator_tolerance_;
    double time_horizon_;
    int discretization_intervals_;

private:

    /**
    * @brief free_allocated_memory: remove all allocated data just for memory management
    */
    void free_allocated_memory();

    /**
    * @brief print_configuration_parameter: debug purpose print set data member of this class
    */
    void print_configuration_parameter();

};

#endif // LMPCC_CONFIGURATION_H
