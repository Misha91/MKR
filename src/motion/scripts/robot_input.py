#!/usr/bin/env python

import csv
import rospy
from robot_class import Robot
import sys
import os

print("We started")
print(os.getcwd())
robots = []

with open('src/motion/data/robot_start_positions.csv', mode='r') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter = ',')
    for row in csv_reader:
        x = float(row[0])
        y = float(row[1])
        theta = float(row[2])
        goal = [float(sys.argv[1]),float(sys.argv[2])]
        new_robot = Robot([x,y,theta],goal)
        robots.append(new_robot)

with open('src/motion/data/robot_data.csv', mode='w') as robot_file:
    robot_writer = csv.writer(robot_file, delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
    for robot in robots:
       robot_writer.writerow([robot.name, robot.start_pose[0], robot.start_pose[1], robot.start_pose[2], robot.goal_pose[0], robot.goal_pose[1]])

for robot in robots:
    print("Robot:", robot.name,"Position:", robot.start_pose, "Goal:", robot.goal_pose)
