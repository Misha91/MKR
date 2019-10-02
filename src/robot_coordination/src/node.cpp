#include "ros/ros.h"
#include "robot/robot.h"
#include <sstream>

int main(int argc, char **argv)
{
  /*initialize ROS node*/
  ros::init(argc, argv, "robot");
  /*create node handle*/
  ros::NodeHandle nh;
  /*create an instance of a robot */
  Robot robot;
  /*create a plan as list of waypoints*/
  std::list<Waypoint> path;
  /* create waypoints on the circle with step of 1 degree, diameter of 1 meter and center on [5,4]*/
/*  for (int i = 0 ; i < 360; i++) {
    Waypoint p1;
    p1.pose.position.x =5+cos(i*M_PI/180);
    p1.pose.position.y =4+sin(i*M_PI/180);
    p1.timepoint = ros::Duration(i*0.1+1);
    path.push_back(p1);
  }
  */
  /*assign a plan to the robot*/
//  robot.setPath(path);
  
  /*set the subscriber of the odometry to the robot*/
  ros::Subscriber odometry_subscriber = nh.subscribe("odom", 100, &Robot::odometryCallback, &robot);
  /*create a publiser of commands*/
  ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist> ("cmd_vel",10);
  /*advertise the service to add a path for a robot*/
  ros::ServiceServer service_add_path = nh.advertiseService("add_path", &Robot::addPathService, &robot);
  /*advertise the service to start the execution of the plan*/
  ros::ServiceServer service_start = nh.advertiseService("start_movement", &Robot::startMovementService, &robot);
  /*advertise the service to stop execution of the plan*/
  ros::ServiceServer service_stop = nh.advertiseService("stop_movement", &Robot::stopMovementService, &robot);
  /*set the publisher to the robot*/
  robot.setCommandPublisher(&cmd_pub);
//  robot.startExecutePath();
  /*start processing messages*/
  ros::spin(); 
  return 0;
}

