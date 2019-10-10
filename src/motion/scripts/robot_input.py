#!/usr/bin/env python

import csv
import rospy
from robot_class import Robot
import sys
import os
from planner import plan

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

"""Reading initial positions and goals of robots"""

with open(path + '/src/motion/data/robot_start_positions.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')
    for i,row in enumerate(csv_reader):
        if i != 0:
            x = int(row[0])
            y = int(row[1])
            theta = float(row[2])
            start = [x,y,theta]
            goal = [int(row[3]),int(row[4])]
            new_robot = Robot(start,goal)
            robots.append(new_robot)

"""Writing data file with names"""

with open(path + '/src/motion/data/robot_data.csv', mode='w') as robot_file:
    robot_writer = csv.writer(robot_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for robot in robots:
       robot_writer.writerow([robot.name, robot.start_pose[0], robot.start_pose[1], robot.start_pose[2], robot.goal_pose[0], robot.goal_pose[1]])

"""Not necessary, use robots list directly"""
#robot_list = []
#for robot in robots:
#    tmp = Robot(robot.start_pose, robot.goal_pose)
#    robot_list.append(tmp)
#    print("Robot:", robot.name,"Position:", robot.start_pose, "Goal:", robot.goal_pose)

"""Getting map"""
rospy.init_node("path_planner")
init_map = rospy.wait_for_message("/map", OccupancyGrid)
#print(init_map)
map = np.reshape(init_map.data, (init_map.info.height, init_map.info.width)).T
map = skimage.measure.block_reduce(map, (20,20), np.max)
print(map.shape)

"""Print robot data"""
for robot in robots:
    print(robot.name,robot.start_pose, robot.goal_pose)


#self.grid = (morphology.grey_dilation(self.grid, size=(3,3)))
#robot_list_updated = plan(map, robot_list)
print(robots[0].start_pose[:2])
print(robots[0].goal_pose)
robot_path = plan(map, robots[0].start_pose[:2],robots[0].goal_pose)
print("Robot zero path")
print(robot_path)


"""PRINTING OF MAP"""
for j in (reversed(range(map.shape[1]))):
    str = ""
    for i in range(map.shape[0]):
        if (map[i][j] >0) or ([i,j] in robot_path): str += "#"
        else: str += " "
    print(str)
print(map[10][30])
print("UPDATED")
