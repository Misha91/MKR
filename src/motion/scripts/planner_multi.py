#!/usr/bin/env python

from planner import plan
from scipy.ndimage import morphology
from utils import manhattan_dist, insort
import numpy as np
# input list of robot objects with their start position and goal position
# output list of waypoints for each robot

def multi_plan(map,robots):
    try:
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

        finished = False
        list_of_maps = []
        for robot in robots:

            start = robot.start_pose[:2]
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

        """ map with starting positions"""
        map_now = np.copy(map_init)

        for robot in robots:
            map_other_robots = np.copy(map_now)
            start = robot.start_pose[:2]
            map_other_robots[start[0]][start[1]] = 0
            robot.map = np.copy(map_other_robots)
            robot.map = morphology.grey_dilation(robot.map, size=(3,3))

        while(finished == False):

            for i,robot in enumerate(robots):

                if robot.is_finished == False:

                    parent_pose = robot.new_parent[1]
                    parent_path = robot.new_parent[2]
                    ### creating new nodes
                    for x in list_of_directions:
                        """node creation"""
                        history = list(parent_path) + [[parent_pose[0]+x[0],parent_pose[1]+x[1]]]
                        new_node = [manhattan_dist([parent_pose[0]+x[0],parent_pose[1]+x[1]], goal) + len(history), #D
                                        [parent_pose[0]+x[0],parent_pose[1]+x[1]], # new pose
                                        history #path
                                        ]
                        # print(robot.priority_queue)
                        """filtering"""
                        ### within map bounds ###
                        if new_node[1][0]<0 or new_node[1][0]>x_bound:
                            continue
                        if new_node[1][1]<0 or new_node[1][1]>y_bound:
                            continue

                        ### not occupied ###

                        if robot.map[new_node[1][0]][new_node[1][1]]>occupancy_threshold:
                            continue

                        ### not considered before ###
                        if new_node[1] in robot.priority_poses and len(new_node[2]) in robot.priority_lens:
                            continue

                        """adding to queue"""
                        if len(new_node)>0:
                            robot.priority_queue = insort(robot.priority_queue, new_node)
                            robot.priority_poses.append(new_node[1])
                            robot.priority_lens.append(len(new_node[2]))

                    """selection of new parent node"""

                    robot.new_parent = robot.priority_queue[0]

                    robot.priority_queue.pop(0)
                    robot.priority_poses.pop(robot.priority_poses.index(robot.new_parent[1]))
                    robot.priority_lens.pop(len(robot.new_parent[2]))
                    print(robot.new_parent)
                """if finished"""
                if robot.new_parent[1]==robot.goal_pose:
                    # print("working")
                    robot.is_finished = True
                    robot_fin[i] = True
                    robot.waypoint = robot.new_parent[2]
                robot_x = robot.new_parent[1][0]
                robot_y = robot.new_parent[1][1]
                map_now[robot_x][robot_y] = 100
                """update map"""

            # clear map
            map_now = np.copy(map_init)
            # insert robots
            for robot in robots:
                map_now[robot.new_parent[1][0]][robot.new_parent[1][1]] = 100
            # morph
            for robot in robots:
                map_other_robots = np.copy(map_now)
                pose = robot.new_parent[1]
                map_other_robots[pose[0]][pose[1]] = 0
                robot.map = np.copy(map_other_robots)
                robot.map = morphology.grey_dilation(robot.map, size=(3,3))

            finished = all(x == True for x in robot_fin)

        return robots
    except Exception as e:
        print(e)
        return False
