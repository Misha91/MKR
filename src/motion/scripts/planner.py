#!/usr/bin/env python

from utils import insort, manhattan_dist

def plan(map, start, goal):

        visited_nodes = []
        visited_poses = []
        priority_queue = []
        priority_poses = []
        priority_lens = []
        x_bound = map.shape[0]
        y_bound = map.shape[1]
        occupancy_threshold = 0

        if goal[0]<0 or goal[0]>x_bound:
            raise Exception("Goal is out of x bound")
        if goal[1]<0 or goal[1]>y_bound:
            raise Exception("Goal is out of y bound")
        if map[goal[0]][goal[1]]>occupancy_threshold:
            raise Exception("Goal is occupied")
        if map[goal[0]][goal[1]]<0:
            raise Exception("Goal is in unexplored region")

        """configuration of node search"""
        list_of_directions = [[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1],[0,0]]

        """initialization"""
        ### building starting node
        ### node = [D, pose, path]
        init_dist = manhattan_dist(start,goal)
        start_node = [init_dist, start, [start]]
        new_parent = start_node

        """node iterations"""
        while(new_parent[1]!=goal):

            parent_pose = new_parent[1]
            parent_path = new_parent[2]

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
                if map[new_node[1][0]][new_node[1][1]]>occupancy_threshold:
                    continue

                ### not considered before ###

                if new_node[1] in priority_poses and len(new_node[2]) in priority_lens:
                    continue

                """adding to queue"""
                priority_queue = insort(priority_queue, new_node)
                priority_poses.append(new_node[1])
                priority_lens.append(len(new_node[2]))

            """selection of new parent node"""
            new_parent = priority_queue[0]

            priority_queue.pop(0)
            priority_poses.pop(priority_poses.index(new_parent[1]))
            priority_lens.pop(len(new_parent[2]))


        """return path"""
        return (new_parent[2])
