#include <cmath>  // For std::{cos,sin}.
#include <random>  // For std::{random_device,default_random_engine,uniform_real_distribution,normal_distribution}.

#include <nav_msgs/GetMap.h>

#include <particle_filter_ros/particle_filter.h>

namespace particle_filter_ros
{

std::random_device random_device;
std::default_random_engine random_engine{random_device()};
std::uniform_real_distribution<double> uniform_distribution{0, 1};
std::normal_distribution<double> normal_distribution{0, 1};

/** Return one sample of a uniform distribution between min and max
 */
double randBetween(double min, double max)
{
  return min + uniform_distribution(random_engine) * (max - min);
}

/** Return one sample of a normal distribution with the given mean and standard deviation
 */
double nrand(double mean, double sigma)
{
  return mean + normal_distribution(random_engine) * sigma;
}

inline
RobotPosition transformedPosition(double x, double y, double x0, double y0, double yaw)
{
  RobotPosition pos;
  const double cos_yaw = std::cos(yaw);
  const double sin_yaw = std::sin(yaw);
  pos.x = x0 + x * cos_yaw - y * sin_yaw;
  pos.y = y0 + x * sin_yaw + y * cos_yaw;
  pos.phi = yaw;
  return pos;
}

ParticleFilter::ParticleFilter()
{
  getMap();

  laser_simulator_ptr_.reset(new LaserSimulator{map_});

}

/** Get the map from ROS service /static_map
 */
bool ParticleFilter::getMap()
{
  ros::NodeHandle nh;

  ros::ServiceClient static_map_client = nh.serviceClient<nav_msgs::GetMap>("static_map");

  while (!static_map_client.waitForExistence(ros::Duration(5)))
  {
    ROS_WARN_STREAM("Waiting for service /static_map");
  }

  ROS_DEBUG("server found");

  nav_msgs::GetMap get_map_srv;
  if (!static_map_client.call(get_map_srv))
  {
    ROS_ERROR("Failed to call service /static_map");
    return false;
  }
  ROS_DEBUG("server called");

  map_ = get_map_srv.response.map;

  ROS_DEBUG("Got a map");

  return true;
}

/** Generate a set of random of equally weighted particles
 *
 * The weight of each particle will be 1 / count.
 */
ParticleVector ParticleFilter::generateRandomParticles(unsigned int count) const
{
  ParticleVector particles;
  particles.resize(count);
  const double min_x = 0;
  const double min_y = -map_.info.height * map_.info.resolution;
  const double max_x = map_.info.width * map_.info.resolution;
  const double max_y = 0;
  for (size_t i = 0; i < count; i++)
  {
    const double x = randBetween(min_x, max_x);
    const double y = randBetween(min_y, max_y);
    const double phi = randBetween(-M_PI, M_PI);
    Particle p;
    p.pos = RobotPosition(transformedPosition(map_.info.origin.position.x, map_.info.origin.position.y, x, y, phi));
    if (laser_simulator_ptr_->isFeasible(p.pos))
    {
      p.weight = 1.0 / static_cast<double>(count);
      particles.push_back(p);
      i++;
    }
  }
  return particles;
}

} /* namespace particle_filter_ros */
