#!/usr/bin/env python

import csv
import rospy
from robot_class import Robot
import sys
import os
import planner

from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.ndimage import morphology
import skimage.measure

print("We started")
path = os.getcwd()
robots = []
if (path.split('/')[-1] != 'mkr_tm_ws'):
    path += '/mkr_tm_ws'

print(path)
with open(path + '/src/motion/data/robot_start_positions.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')
    for row in csv_reader:
        x = float(row[0])
        y = float(row[1])
        theta = float(row[2])
        goal = [float(sys.argv[1]),float(sys.argv[2])]
        new_robot = Robot([x,y,theta],goal)
        robots.append(new_robot)

with open(path + '/src/motion/data/robot_data.csv', mode='w') as robot_file:
    robot_writer = csv.writer(robot_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for robot in robots:
       robot_writer.writerow([robot.name, robot.start_pose[0], robot.start_pose[1], robot.start_pose[2], robot.goal_pose[0], robot.goal_pose[1]])


robot_list = []
for robot in robots:
    tmp = Robot(robot.start_pose, robot.goal_pose)
    robot_list.append(tmp)
    print("Robot:", robot.name,"Position:", robot.start_pose, "Goal:", robot.goal_pose)

rospy.init_node("path_planner")
init_map = rospy.wait_for_message("/map", OccupancyGrid)
#print(init_map)

map = np.reshape(init_map.data, (init_map.info.height, init_map.info.width)).T
print(init_map.info.height, init_map.info.width)
map = skimage.measure.block_reduce(map, (10,10), np.max)

print(map.shape)

for robot in robot_list:
    print(robot.start_pose, robot.goal_pose)



"""
for j in range(map.shape[1]):
    str = ""
    for i in range(map.shape[0]):
        if (map[i][j] >0): str += "#"
        else: str += " "
    print(str)
print("UPDATED")
"""

#self.grid = (morphology.grey_dilation(self.grid, size=(3,3)))



#robot_list_updated = planner.plan(map, robot_list)
