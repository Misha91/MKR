#include "robot/waypoint.h"

Waypoint::Waypoint() {
};

Waypoint::Waypoint(geometry_msgs::Pose p, ros::Duration d) {
  pose = p;
  timepoint = d;
}

/** return distance to given pose*distance to given pose*/
double Waypoint::distanceTo(geometry_msgs::Pose p) {
  double d_x = pose.position.x - p.position.x;
  double d_y = pose.position.y - p.position.y;
  double dist = sqrt(d_x*d_x + d_y*d_y);
  return dist; 
}

/** return relative angle to the given pose*/
double Waypoint::angleTo(geometry_msgs::Pose p) {
  double d_x = pose.position.x - p.position.x;
  double d_y = pose.position.y - p.position.y;
  double angle = atan2(d_y, d_x);
  return angle;

}
