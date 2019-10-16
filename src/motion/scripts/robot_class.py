
import os
import rospy
import math

import numpy as np

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


        self.letters = list(ascii_uppercase)
        self.list_of_digits = list(digits)
        self.name = "".join(choice(self.letters)
                            + choice(self.list_of_digits))
        self.pose = (rospy.wait_for_message("/robot" + str(self.id) + "/odom/", Odometry)).pose.pose.position
        if not all(isinstance(x, float) or isinstance(x, int) for x in goal):
            raise Exception("The start, goal and theta positions are misdefined, please use type int or float")
        else:
            self.start_pose = self.coordToGrid(self.pose.x, self.pose.y)
            self.goal_pose = goal
        self.priority_queue = []
        self.priority_poses = []
        self.priority_lens = []
        self.start_node = []
        self.new_parent = []
        self.is_finished = False
        self.waypoint = []

        self.map = []
        self.wp_iter = 1
        tmp_init = self.gridToCoord(self.goal_pose[0],self.goal_pose[1])
        self.goalFinal = Point(tmp_init[0], tmp_init[1], 0)
        self.odomSub = rospy.Subscriber("/robot" + str(self.id) + "/odom/", Odometry, self.odomUpdate)
        self.velPub = rospy.Publisher("/robot" + str(self.id) + "/cmd_vel", Twist, queue_size = 1)
        self.r = rospy.Rate(100)

    def reset(self):
        prev_name = self.name
        new_name = prev_name
        while prev_name == new_name:
            new_name = "".join(choice(self.letters)
                            + choice(self.list_of_digits))
        self.name = new_name

    def odomUpdate(self, msg):
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y
        rot_q = msg.pose.pose.orientation
        (roll, pitch, theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])
        self.pose.z = theta
        #print(self.id, self.pose)

    def coordToGrid(self, x, y):
        return [int(x/self.mult), int(y/self.mult)]

    def gridToCoord(self, x, y):
        return [float(x*self.mult), float(y*self.mult)]

    def getDistance(self, A, B):
        return math.sqrt((A.x-B.x)**2 + (A.y-B.y)**2)

    def goToFinalGoal(self):
        tmp = 0
        while(self.getDistance(self.pose, self.goalFinal) >= 0.05):
            #print("DISTANCE: " + str(self.getDistance(self.pose, self.goalFinal)))
            self.keepGoing()
            #tmp += 1
            #if (tmp >= 3000): break

        self.speed.linear.x = 0.0
        self.speed.angular.z = 0.0
        self.velPub.publish(self.speed)

    def keepGoing(self):

        goal_coord = self.gridToCoord(self.waypoint[self.wp_iter][0], self.waypoint[self.wp_iter][1])
        goal_coord = Point(goal_coord[0], goal_coord[1], 0)
        cur_dist = self.getDistance(self.pose, goal_coord)

        if cur_dist < 0.05:
            if (self.wp_iter < (len(self.waypoint)-1)):
                self.wp_iter += 1
                print("Going to point #" + str(self.wp_iter) + "/" + str(len(self.waypoint)))

        #print(self.wp_iter, self.pose, goal_coord)

        inc_x = goal_coord.x - self.pose.x
        inc_y = goal_coord.y - self.pose.y

        angle_to_goal = math.atan2(inc_y, inc_x)
        diff = angle_to_goal - self.pose.z
        #print(goal_coord, self.pose)
        #print(angle_to_goal, self.pose.z, diff)
        if abs(diff) > 0.1:
            if (self.wp_iter >= 2):
                self.speed.linear.x = 0.2
            self.speed.angular.z = 0.8
            if (diff <= 3):
                self.speed.angular.z = self.speed.angular.z * np.sign(diff)
        else:
            self.speed.linear.x = 0.8
            self.speed.angular.z = 0.0

        self.velPub.publish(self.speed)
        self.r.sleep()


########################################
### You can check if it works ##########
#robot1 = Robot([0,2],[0,2.])
##robot1 = Robot(0.3,"bullshit")
##robot2 = Robot([0,"bullshit"],[5,10])
#robot2 = Robot([0,1],[10,0])
#print("Robot's names", robot1.name , robot2.name)
#robot1.reset()
#robot2.reset()
#print("New robots' names", robot1.name, robot2.name)
#print("Robot 1 goes from", robot1.start_pose, "to ",robot1.goal_pose)
#########################################
#########################################
