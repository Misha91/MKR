#include <cstdlib>  // For EXIT_FAILURE, EXIT_SUCCESS, std::exit.

#include <ros/ros.h>

#include <particle_filter_ros/particle_filter.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "particle_filter");

  ros::NodeHandle private_nh("~");

  particle_filter_ros::ParticleFilter filter;

  ros::spin();
  
  std::exit(EXIT_SUCCESS);
}
