#!/usr/bin/env python

from planner import plan
from scipy.ndimage import morphology
from utils import manhattan_dist
# input list of robot objects with their start position and goal position
# output list of waypoints for each robot

def multi_plan(map,robots):
    list_of_directions = [[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1],[0,0]]

    x_bound = map.shape[0]
    y_bound = map.shape[1]
    occupancy_threshold = 0

    cur_pos_robots = []
    goals = []

    start_nodes = []
    new_parents = []

    for robot in robots:

        start = robot.start_pose
        goal = robot.goal_pose
        map[start[0]][start[1]] = 100


        if goal[0]<0 or goal[0]>x_bound:
            raise Exception("Goal of robot {} is out of x bound".format(robot.name))
        if goal[1]<0 or goal[1]>y_bound:
            raise Exception("Goal of robot {} is out of y bound".format(robot.name))
        if map[goal[0]][goal[1]]>occupancy_threshold:
            raise Exception("Goal of robot {} is occupied".format(robot.name))
        if map[goal[0]][goal[1]]<0:
            raise Exception("Goal of robot {} is in unexplored region".format(robot.name))
        goals.append(goal)

        """initialization"""
        init_dist = manhattan_dist(start,goal)
        start_nodes.append([init_dist, start, [start]])
        new_parents.append(start_node)

    while(finished == False):
        

    map = morphology.grey_dilation(map, size=(3,3))


    ### map object is changed simultaniously in robot_input too, idk why?###


    return map
