#!/usr/bin/env python

import csv
import rospy
from robot_class import Robot
import sys
import os
from planner import plan
from planner_multi import multi_plan

from nav_msgs.msg import OccupancyGrid
import numpy as np
from scipy.ndimage import morphology
import skimage.measure
from matplotlib import colors

import matplotlib.pyplot as plt
print("We started")
path = os.getcwd()
robots = []
if (path.split('/')[-1] != 'mkr_tm_ws'):
    path += '/mkr_tm_ws'

rospy.init_node("path_planner")

"""Getting map"""
pool_kernel = 20
init_map = rospy.wait_for_message("/map", OccupancyGrid)
print(init_map.info.height, init_map.info.width, init_map.info.resolution)
map_res = init_map.info.resolution
map = np.reshape(init_map.data, (init_map.info.height, init_map.info.width)).T
map = skimage.measure.block_reduce(map, (pool_kernel, pool_kernel), np.max)
print(map.shape)
# print(map)

"""Reading initial positions and goals of robots"""

with open(path + '/src/motion/data/robot_start_positions.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')
    for i,row in enumerate(csv_reader):
        if (i == 0): continue
        goal = [int(row[3]),int(row[4])]
        new_robot = Robot(i -1, map_res, pool_kernel, goal)
        #new_robot.pose_coord = [int(new_robot.pose.x/(map_res*pool_kernel)), int(new_robot.pose.y/(map_res*pool_kernel))]
        robots.append(new_robot)

"""Writing data file with names"""

with open(path + '/src/motion/data/robot_data.csv', mode='w') as robot_file:
    robot_writer = csv.writer(robot_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for robot in robots:

        print(int(robot.pose.x/(map_res*pool_kernel)), int(robot.pose.y/(map_res*pool_kernel)))
        robot_writer.writerow([robot.name, robot.start_pose[0], robot.start_pose[1], robot.goal_pose[0], robot.goal_pose[1]])

"""Not necessary, use robots list directly"""
#robot_list = []
#for robot in robots:
#    tmp = Robot(robot.start_pose, robot.goal_pose)
#    robot_list.append(tmp)
#    print("Robot:", robot.name,"Position:", robot.start_pose, "Goal:", robot.goal_pose)

"""Print robot data"""
for robot in robots:
    print(robot.name,robot.start_pose, robot.goal_pose)


"""Planning"""


"""
#Test waypoint
morph_size = 3

morph_map = morphology.grey_dilation(map, size=(morph_size,morph_size))

robots[1].waypoint = plan(morph_map, robots[1].start_pose, robots[1].goal_pose)
print("Robot path")
print(robots[1].waypoint)
robot_path = robots[1].waypoint

#map = multi_plan(map, robots)
#PRINTING OF MAP

for j in (reversed(range(map.shape[1]))):
    str = ""
    for i in range(map.shape[0]):
        if (map[i][j] >0) or ([i,j] in robot_path):
            str += "#"
        else: str += " "
    print(str)
print("UPDATED")

robots[1].goToFinalGoal()
#end of testing waypoint
"""

# morph_size = 3

# morph_map = morphology.grey_dilation(map, size=(morph_size,morph_size))
# robot_path = plan(morph_map, robots[0].start_pose[:2],robots[0].goal_pose)
# print("Robot path")
# print(robot_path)
#
# list_of_pathes = []
# new_robots = multi_plan(map, robots)
# for robot in new_robots:
#      print(robot.waypoint)
#      list_of_pathes.append(robot.waypoint)
#
# list_of_pathes.sort(lambda x,y: len(x) < len(y))
#
# plt.ion()
# plt.show()
# p = []
# cmap = colors.ListedColormap(['white','black'])
# for i in range(len(list_of_pathes[0])):
#     new_map = np.copy(map)
#     # if i < len(list_of_pathes[-1]):
#     #     for robot in new_robots:
#     #         p[robot.waypoint[i][0]][robot.waypoint[i][1]] = 100
#     # plt.plot(list_of_pathes[0][i][0],list_of_pathes[0][i][1],'bo')
#     # plt.plot(list_of_pathes[1][i][0],list_of_pathes[1][i][1],'r+')
#
#
#     if i < len(list_of_pathes[-1]):
#         new_map[list_of_pathes[1][i][0]][list_of_pathes[1][i][1]]= 100
#     new_map[list_of_pathes[0][i][0],list_of_pathes[0][i][1]] = 100
#     plt.imshow(new_map,cmap = cmap)
#
#     plt.pause(0.5)

# """PRINTING OF MAP"""
#
# for j in (reversed(range(map.shape[1]))):
#     str = ""
#     for i in range(map.shape[0]):
#         if (map[i][j] >0) or ([i,j] in robot_path):
#             str += "#"
#         else: str += " "
#     print(str)
# print("UPDATED")
""" MULTI-PLANNING """
# input: map, robot list
# output robot list with modified self.waypoint
planned_robots = multi_plan(map,robots)
for robot in planned_robots:
    print(robot.waypoint)
