from string import ascii_uppercase, digits
from random import choice
import os

class Robot(object):
    def __init__(self, start, goal):
        self.letters = list(ascii_uppercase)
        self.list_of_digits = list(digits)
        self.name = "".join(choice(self.letters)
                            + choice(self.list_of_digits))

        if not all(isinstance(x, float) or isinstance(x,int) for x in start) or not all(isinstance(x, float) or isinstance(x, int) for x in goal):
            raise Exception("The start, goal and theta positions are misdefined, please use type int or float")
        else:
            self.start_pose = start
            self.goal_pose = goal
        self.priority_queue = []
        self.priority_poses = []
        self.priority_lens = []
        self.start_node = []
        self.new_parent = []
        self.is_finished = False
        self.waypoint = []
        self.map = []

    def reset(self):
        prev_name = self.name
        new_name = prev_name
        while prev_name == new_name:
            new_name = "".join(choice(self.letters)
                            + choice(self.list_of_digits))
        self.name = new_name


########################################
### You can check if it works ##########
#robot1 = Robot([0,2],[0,2.])
##robot1 = Robot(0.3,"bullshit")
##robot2 = Robot([0,"bullshit"],[5,10])
#robot2 = Robot([0,1],[10,0])
#print("Robot's names", robot1.name , robot2.name)
#robot1.reset()
#robot2.reset()
#print("New robots' names", robot1.name, robot2.name)
#print("Robot 1 goes from", robot1.start_pose, "to ",robot1.goal_pose)
#########################################
#########################################
