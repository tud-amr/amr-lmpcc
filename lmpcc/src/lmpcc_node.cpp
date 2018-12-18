#include <ros/ros.h>
#include <lmpcc/lmpcc_controller.h>

int main(int argc, char **argv)
{
  try
  {
    ros::init(argc, argv, ros::this_node::getName());
    LMPCC controller_;

    // initialize predictive control node
    if (!controller_.initialize())
    {
      ROS_ERROR_STREAM_NAMED("FAILED TO INITIALIZE %s", ros::this_node::getName().c_str());
      exit(1);
    }
    else
    {
      // spin node, till ROS node is running on
      ROS_INFO_STREAM_NAMED("%s INITIALIZE SUCCESSFULLY!!", ros::this_node::getName().c_str());
      ros::spin();
    }
  }

  catch (ros::Exception& e)
  {
    ROS_ERROR("predictive_control_node: Error occured: %s ", e.what());
    exit(1);
  }

  return 0;
}
