#!/usr/bin/env python
# -*- encoding: utf-8 -*-
# Example of script to send a trajectory to a robot, start and stop motion.

import time

from geometry_msgs.msg import Pose
from robot_coordination.msg import Waypoint
from robot_coordination.srv import AddPath
from robot_coordination.srv import StartMovement
from robot_coordination.srv import StopMovement
import rospy



def wait_for_service(server_ns, srv_name):
    while not rospy.is_shutdown():
        try:
            rospy.wait_for_service(server_ns + '/' + srv_name, 5)
            return True
        except rospy.ROSException:
            rospy.logwarn('Could not connect to service {}/{}, trying again'.format(
                server_ns, srv_name))
        except (rospy.ROSInterruptException, KeyboardInterrupt):
            return False


def start_movement(server_ns):
    """Start the robot motion by calling the service 'start_movement'"""
    srv_name = 'start_movement'
    if not wait_for_service(server_ns, srv_name):
        return False
    try:
        start_movement_srv = rospy.ServiceProxy(server_ns + '/' + srv_name, StartMovement)
        reply = start_movement_srv()
        return reply.ack
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))
    return False


def stop_movement(server_ns):
    """Stop the robot motion by calling the service 'stop_movement'"""
    srv_name = 'stop_movement'
    if not wait_for_service(server_ns, srv_name):
        return False
    try:
        stop_movement_srv = rospy.ServiceProxy(server_ns + '/' + srv_name, StopMovement)
        reply = stop_movement_srv()
        return reply.ack
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))
    return False


def waypoint_list_from_tuple(traj):
    """Return a list of Waypoint instances from a list of (x, y, timepoint)"""
    waypoint_list = []
    for x, y, timepoint in traj:
        waypoint = Waypoint()
        waypoint.pose = Pose()
        waypoint.pose.position.x = x
        waypoint.pose.position.y = y
        waypoint.timepoint = rospy.Duration.from_sec(timepoint)
        waypoint_list.append(waypoint)
    return waypoint_list


def add_path(server_ns, trajectory):
    """Add a path with data from trajectory"""
    waypoint_list = waypoint_list_from_tuple(trajectory)
    srv_name = 'add_path'
    if not wait_for_service(server_ns, srv_name):
        return False
    try:
        add_path_srv = rospy.ServiceProxy(server_ns + '/' + srv_name, AddPath)
        reply = add_path_srv(waypoint_list)
        return reply.result
    except rospy.ServiceException as e:
        rospy.logerr('Service call failed: {}'.format(e))
    return False
