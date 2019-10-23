#!/usr/bin/env python

import csv
import rospy
from robot_class import Robot
import sys
import os
import subprocess
from planner import plan
from planner_multi import multi_plan
from coord_helper import *


from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.ndimage import morphology
import skimage.measure
from matplotlib import colors

import matplotlib.pyplot as plt
print("We started")

path = (subprocess.check_output("cd ../data; pwd", shell=True)).rstrip()

robots = []

print(path)
rospy.init_node("path_planner")

"""Getting map"""
pool_kernel = 20
init_map = rospy.wait_for_message("/map", OccupancyGrid)
print(init_map.info.height, init_map.info.width, init_map.info.resolution)
map_res = init_map.info.resolution
map = np.reshape(init_map.data, (init_map.info.height, init_map.info.width)).T
map = skimage.measure.block_reduce(map, (pool_kernel, pool_kernel), np.max)
print(map.shape)

"""Reading initial positions and goals of robots"""

with open(path + '/robot_start_positions.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')
    for i,row in enumerate(csv_reader):
        if (i == 0): continue
        goal = [int(row[3]),int(row[4])]
        new_robot = Robot(i -1, map_res, pool_kernel, goal)
        robots.append(new_robot)

"""Writing data file with names"""

with open(path + '/robot_data.csv', mode='w') as robot_file:
    robot_writer = csv.writer(robot_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for robot in robots:

        print(int(robot.pose.x/(map_res*pool_kernel)), int(robot.pose.y/(map_res*pool_kernel)))
        robot_writer.writerow([robot.name, robot.start_pose[0], robot.start_pose[1], robot.goal_pose[0], robot.goal_pose[1]])

"""Print robot data"""
for robot in robots:
    print(robot.name,robot.start_pose, robot.goal_pose)

""" MULTI-PLANNING """
# input: map, robot list
# output robot list with modified self.waypoint
planned_robots = multi_plan(map,robots)
for robot in planned_robots:
    print(robot.waypoint)
    robot.startMoving()

for robot in planned_robots:
    start_movement(robot.nameId)
