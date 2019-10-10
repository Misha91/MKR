from utils import insort
def plan(map, start, goal):
        visited_nodes = []
        priority_queue = []
        x_bound = map.shape[0]
        y_bound = map.shape[1]
        occupancy_threshold = 0

        """configuration of node search"""
        robot_radius = 1

        list_of_directions = [[0,robot_radius],[robot_radius,robot_radius],
        [robot_radius,0],[robot_radius,-robot_radius],
        [0,-robot_radius],[-robot_radius,-robot_radius],
        [-robot_radius,0],[-robot_radius,robot_radius]]


        """initialization"""
        ### building starting node
        init_dist = manhattan_dist(start,goal)
        ### node = [D, pose, path]
        start_node = [init_dist, start, [start]]
        visited_nodes.append(start_node)

        """loop"""
        while(visited_nodes[0][1]!=goal):
            path = visited_nodes[0][2]
            current_pose = visited_nodes[0][1]
            ### creating new nodes
            for x in list_of_directions:
                """node creation"""
                new_node = [[manhattan_dist([current_pose[0]+x[0],current_pose[1]+x[1]], goal), #D
                                [current_pose[0]+x[0],current_pose[1]+x[1]], # new pose
                                list(path) + [[current_pose[0]+x[0],current_pose[1]+x[1]]] #path
                                ] ]
                """filtering"""
                ### within map bounds ###
                if new_node[1][0]<0 or new_node[1][0]>x_bound:
                    continue
                if new_node[1][1]<0 or new_node[1][1]>y_bound:
                    continue
                ### not occupied ###
                if map[new_node[1][0]][new_node[1][1]]>occupancy_threshold:
                    continue

                """adding to queue"""
                insort(priority_queue, new_node)



def manhattan_dist(start, goal):
    dist = abs(goal[0]-start[0])+abs(goal[1]-start[1])
    return dist


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
