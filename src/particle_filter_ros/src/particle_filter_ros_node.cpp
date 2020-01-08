#include <cstdlib>  // For EXIT_FAILURE, EXIT_SUCCESS, std::exit.

#include <ros/ros.h>
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/LaserScan.h"
#include <stdio.h>
#include <particle_filter_ros/particle_filter.h>
#include <string>
#include <limits.h>
#include <unistd.h>
#include <vtkPNGReader.h>
#include "gui/gui.h"
#include "typedefs.h"

using namespace imr;
using namespace gui;
//using namespace laserDataLoader;

std::string getexepath()
{
  char result[ PATH_MAX ];
  ssize_t count = readlink( "/proc/self/exe", result, PATH_MAX );
  return std::string( result, (count > 0) ? count : 0 );
}

void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
int main(int argc, char *argv[])
{
  printf("%s", getexepath().c_str());
  //ros::init(argc, argv, "particle_filter");

  //ros::NodeHandle n;

  // particle_filter_ros::ParticleFilter filter;
  //
  // ros::Subscriber sub = n.subscribe("odom", 1000, chatterCallback);
  // ros::Subscriber sub_las = n.subscribe("scan", 1000, laserCallback);
  vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
  reader->SetFileName("src/area.png");
  Gui gui(reader);
  gui.startInteractor();

  ros::spin();

  std::exit(EXIT_SUCCESS);
}
void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
  // printf("moving");
  // ROS_INFO("Seq: [%d]", msg->header.seq);
  // ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", msg->pose.pose.position.x,msg->pose.pose.position.y, msg->pose.pose.position.z);
  // ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
  // ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", msg->twist.twist.linear.x,msg->twist.twist.angular.z);
}
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // printf("sensing");
  // ROS_INFO("Seq: [%d]", msg->header.seq);
}
