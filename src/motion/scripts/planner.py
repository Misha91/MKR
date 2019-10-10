#!/usr/bin/env python


from utils import insort, manhattan_dist

def plan(map, start, goal):

        visited_nodes = []
        priority_queue = []
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
            raise Exception("Goal is out of map")


        """configuration of node search"""
        robot_radius = 1

        list_of_directions = [[0,robot_radius],[robot_radius,robot_radius],
        [robot_radius,0],[robot_radius,-robot_radius],
        [0,-robot_radius],[-robot_radius,-robot_radius],
        [-robot_radius,0],[-robot_radius,robot_radius]]


        """initialization"""
        ### building starting node
        ### node = [D, pose, path]
        init_dist = manhattan_dist(start,goal)
        start_node = [list([init_dist]), start, [start]]

        visited_nodes.append(start_node)

        """loop"""
        while(visited_nodes[-1][1]!=goal):

            node_path = visited_nodes[-1][2]
            current_pose = visited_nodes[-1][1]
            ### creating new nodes
            for x in list_of_directions:
                """node creation"""
                new_node = [list([manhattan_dist([current_pose[0]+x[0],current_pose[1]+x[1]], goal)]), #D
                                [current_pose[0]+x[0],current_pose[1]+x[1]], # new pose
                                list(node_path) + [[current_pose[0]+x[0],current_pose[1]+x[1]]] #path
                                ]
                """filtering"""
                #old = False
                if new_node in visited_nodes:
                    continue
                ### within map bounds ###
                if new_node[1][0]<0 or new_node[1][0]>x_bound:
                    continue
                if new_node[1][1]<0 or new_node[1][1]>y_bound:
                    continue
                ### not occupied ###
                if map[new_node[1][0]][new_node[1][1]]>occupancy_threshold:
                    continue
                """
                for old_node in visited_nodes:
                    if new_node[1] == old_node[1]:
                        old = True
                        break
                if old:
                    continue

                if map[new_node[1][0]][new_node[1][1]]<0:
                    continue
                """

                """adding to queue"""
                print(new_node[1])
                insort(priority_queue, new_node)

            visited_nodes.append(priority_queue[0])
            priority_queue.pop(0)


        """return path"""
        return (visited_nodes[-1][2])






"""
list_of_directions = [[0,1],[1,1],[1,0],[1,-1],[0,-1],[-1,-1],[-1,0],[-1,1]]
visited_nodes = [[5,[0,0],[0,0]]]
#while(visited_nodes[0][1]!=goal):
goal = [10,10]
path = [visited_nodes[0][2]]
node = visited_nodes[0][1]
print(path)
new_nodes = [[manhattan_dist([node[0]+x[0],node[1]+x[1]], goal),
                [node[0]+x[0],node[1]+x[1]],
                list(path) + [[node[0]+x[0],node[1]+x[1]]]  ] for x in list_of_directions]
"""
