#!/usr/bin/env python

from planner import plan
from scipy.ndimage import morphology
from utils import manhattan_dist
import numpy as np
# input list of robot objects with their start position and goal position
# output list of waypoints for each robot

def multi_plan(map,robots):

    map_init = np.copy(map)

    list_of_directions = [[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1],[0,0]]

    x_bound = map.shape[0]
    y_bound = map.shape[1]
    occupancy_threshold = 0

    cur_pos_robots = []
    goals = []

    start_nodes = []
    new_parents = []
    robot_fin = []

    for robot in robots:

        start = robot.start_pose
        goal = robot.goal_pose
        map_init[start[0]][start[1]] = 100
        robot_fin.append(robot.is_finished)

        if goal[0]<0 or goal[0]>x_bound:
            raise Exception("Goal of robot {} is out of x bound".format(robot.name))
        if goal[1]<0 or goal[1]>y_bound:
            raise Exception("Goal of robot {} is out of y bound".format(robot.name))
        if map_init[goal[0]][goal[1]]>occupancy_threshold:
            raise Exception("Goal of robot {} is occupied".format(robot.name))
        if map_init[goal[0]][goal[1]]<0:
            raise Exception("Goal of robot {} is in unexplored region".format(robot.name))
        goals.append(goal)

        """initialization"""
        init_dist = manhattan_dist(start,goal)
        robot.start_node = [init_dist, start, [start]]
        robot.new_parent = robot.start_node


    map_now = np.copy(map_init)
    map_now = morphology.grey_dilation(map_now, size=(3,3))

    while(finished == False):
        for i,robot in enumerate(robots):

            parent_pose = new_parents[i][1]
            parent_path = new_parents[i][2]
            ### creating new nodes
            for x in list_of_directions:
                """node creation"""
                history = list(parent_path) + [[parent_pose[0]+x[0],parent_pose[1]+x[1]]]
                new_node = [manhattan_dist([parent_pose[0]+x[0],parent_pose[1]+x[1]], goal) + len(history), #D
                                [parent_pose[0]+x[0],parent_pose[1]+x[1]], # new pose
                                history #path
                                ]
                """filtering"""
                ### within map bounds ###
                if new_node[1][0]<0 or new_node[1][0]>x_bound:
                    continue
                if new_node[1][1]<0 or new_node[1][1]>y_bound:
                    continue
                ### not occupied ###
                if map_now[new_node[1][0]][new_node[1][1]]>occupancy_threshold:
                    continue
                ### not considered before ###
                if new_node[1] in robot.priority_poses and len(new_node[2]) in robot.priority_lens:
                    continue

                """adding to queue"""
                robot.priority_queue = insort(robot.priority_queue, new_node)
                robot.priority_poses.append(new_node[1])
                robot.priority_lens.append(len(new_node[2]))

            """selection of new parent node"""
            robot.new_parent = robot.priority_queue[0]

            robot.priority_queue.pop(0)
            robot.priority_poses.pop(priority_poses.index(robot.new_parent[1]))
            robot.priority_lens.pop(len(robot.new_parent[2]))

            """if finished"""
            if robot.new_parent[1]==robot.goal_pose:
                robot.is_finished = True

        """update map"""
        # clear map
        map_now = np.copy(map_init)
        # insert robots
        for i,robot in enumerate(robots):
            map_now[robot.new_parent[1][0]][new_parent[1][1]] = 100
        # morph
        map_now = morphology.grey_dilation(map_now, size=(3,3))

        if all(robot_fin) == True:
            finished = True







    return map_init
