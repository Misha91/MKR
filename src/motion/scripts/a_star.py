
from utils import insort, manhattan_dist
import numpy as np

def plan_a_star(map, start, goal):
### Basic A star
        try:
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
        except:
            print("Cannot plan A* basic")

def plan_a_star_3D(map3D, start, goal):
### Modified A* that creates nodes according to next map frame
### input: map3D - map frames in time, start pose, goal pose
        try:
            priority_queue = []
            priority_poses = []
            priority_lens = []
            x_bound = map3D.shape[1]
            y_bound = map3D.shape[2]
            map_range = map3D.shape[0]
            occupancy_threshold = 0
            extension = 0

            if goal[0]<0 or goal[0]>x_bound:
                raise Exception("Goal is out of x bound")
            if goal[1]<0 or goal[1]>y_bound:
                raise Exception("Goal is out of y bound")
            if map3D[0,goal[0],goal[1]]>occupancy_threshold:
                raise Exception("Goal is occupied")
            if map3D[0,goal[0],goal[1]]<0:
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

                current_map_num = len(parent_path)
                ## extension of the 3D map
                # print(map3D.shape)
                if current_map_num == (map_range-2):
                    current_map = np.copy(map3D[-1,:,:])
                    map3D = np.append(map3D,[current_map],axis=0)
                    map_range = map3D.shape[0]
                    extension = extension + 1
                    # print("extended")
                    # print(map3D.shape)
                ### creating new nodes
                for x in list_of_directions:

                    """node creation"""
                    history = list(parent_path) + [[parent_pose[0]+x[0],parent_pose[1]+x[1]]]

                    next_map_num = current_map_num+1
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
                    ### if range is out extend 3D map with the last map
                    if map3D[next_map_num,new_node[1][0],new_node[1][1]]>occupancy_threshold:
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

            return (new_parent[2]),extension
        except:
            print("Cannot plan A* 3D")
