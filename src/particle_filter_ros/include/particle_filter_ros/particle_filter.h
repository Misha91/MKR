#pragma once

#include <memory>  // For std::unique_ptr.

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <particle_filter_ros/laser_simulator.h>
#include <particle_filter_ros/typedefs.h>

namespace particle_filter_ros
{

class ParticleFilter
{
  public:

    ParticleFilter();

    ParticleVector generateRandomParticles(unsigned int count) const;

  protected:

    bool getMap();

    nav_msgs::OccupancyGrid map_;


  private:

  std::unique_ptr<LaserSimulator> laser_simulator_ptr_;
};

} /* namespace particle_filter_ros */
