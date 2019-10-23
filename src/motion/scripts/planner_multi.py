from a_star import plan_a_star, plan_a_star_3D
from scipy.ndimage import morphology
from utils import obtain_rank
import numpy as np

def multi_plan(map,robots):
### The function takes map and robot objects to plan them and returns
### them with their paths at their corresponding waypoint parameter


    morph_size = 3
    morph_size_rob = 7
    morph_coef = 25
    morph_map = morphology.grey_dilation(map, size=(morph_size,morph_size))
    rob_paths = []
# A star for all robots, to find the shortest path
    for robot in robots:
        path = plan_a_star(morph_map, robot.start_pose, robot.goal_pose)
        rob_paths.append(path)
    rob_paths_lens = [len(x) for x in rob_paths]
    rob_ranks = obtain_rank(rob_paths_lens)

# define robot and his path

    main_rob_index = rob_ranks.index(0)
    main_rob_path = rob_paths[main_rob_index]
    # send the path to main robot
    robots[main_rob_index].waypoint = main_rob_path

# prepare map for next robots

    map_to_pass = np.array([map])
    map_to_save = np.array([map])
    empty_map = np.zeros((map.shape[0],map.shape[1]))

    for i,pose in enumerate(main_rob_path):
        temp_map = np.copy(empty_map)
        temp_map[pose[0]][pose[1]] = 100
        temp_map_morph = morphology.grey_dilation(temp_map, size=(morph_size_rob+int(i/morph_coef),morph_size_rob+int(i/morph_coef)))

        map_to_pass = np.append(map_to_pass,[temp_map_morph+morph_map],axis=0)
        map_to_save = np.append(map_to_save,[temp_map+map],axis=0)

# Planning of next robots using modified A *

    for i in range(len(rob_ranks)):
        if (i == 0): continue
        #define
        cur_rob_index = rob_ranks.index(i)
        cur_rob_path, extension = plan_a_star_3D(map_to_pass, robots[cur_rob_index].start_pose, robots[cur_rob_index].goal_pose)
        robots[cur_rob_index].waypoint = cur_rob_path
        #prepare
        for j in range(extension):
            temp_map = np.copy(map_to_save[-1,:,:])
            map_to_save = np.append(map_to_save,[temp_map],axis =0)

        map_to_pass = np.copy(map_to_save)
        for j,pose in enumerate(cur_rob_path):
            temp_map = np.copy(empty_map)
            temp_map[pose[0]][pose[1]] = 100
            temp_map_morph = morphology.grey_dilation(temp_map, size=(morph_size_rob+int(j/morph_coef),morph_size_rob+int(j/morph_coef)))

            map_to_pass[j+1,:,:] = np.copy(temp_map_morph+map)
            map_to_save[j+1,pose[0],pose[1]]=100

        # for j in range(map_to_pass.shape[0]):
            # temp_map_morph = morphology.grey_dilation(map_to_pass[j,:,:],size=(morph_size,morph_size))
            # map_to_pass[j,:,:] = np.copy(temp_map)

    return robots
