
import os
import rospy
import math

import numpy as np
from coord_helper import *
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist

from string import ascii_uppercase, digits
from random import choice


class Robot(object):
    def __init__(self, id, map_res, pool_kernel, goal):
        self.id = id
        self.mult = map_res * pool_kernel
        self.speed = Twist()
        self.nameId = "/robot" + str(id)

        self.letters = list(ascii_uppercase)
        self.list_of_digits = list(digits)
        self.name = "".join(choice(self.letters)
                            + choice(self.list_of_digits))
        self.pose = (rospy.wait_for_message(self.nameId + "/odom/", Odometry)).pose.pose.position
        if not all(isinstance(x, float) or isinstance(x, int) for x in goal):
            raise Exception("The start, goal and theta positions are misdefined, please use type int or float")
        else:
            self.start_pose = self.coordToGrid(self.pose.x, self.pose.y)
            self.goal_pose = goal

        self.waypoint = []
        rospy.set_param('~' + self.nameId, self.nameId)
        self.map = []

    def reset(self):
        prev_name = self.name    #thread.start_new_thread(start_movement, (robot.nameId))
        new_name = prev_name
        while prev_name == new_name:
            new_name = "".join(choice(self.letters)
                            + choice(self.list_of_digits))
        self.name = new_name

    def coordToGrid(self, x, y):
        return [int(x/self.mult), int(y/self.mult)]

    def gridToCoord(self, coord, time):
        return (float(coord[0]*self.mult), float(coord[1]*self.mult), time)


    def startMoving(self):
        traj = []
        for ind, p in enumerate(self.waypoint):
            traj.append(self.gridToCoord(p, ind))
        traj = tuple(traj)
        #print(traj)
        server_ns_parameter_name = "~" + self.nameId

        try:
            server_ns = rospy.get_param(server_ns_parameter_name)
        except KeyError:
            rospy.logerr('Parameter {} must be set'.format(
                rospy.resolve_name(server_ns_parameter_name)))
            return

        if not add_path(server_ns, traj):
            rospy.logerr('Could not add path, exiting')
            return
        rospy.loginfo('Path added')
