from .robomaster_env import *
from copy import deepcopy
from robomaster_gym.misc.geometry import distance as L2
from numpy.random import random

import roslaunch

class CloseQuarter(RobomasterEnv):
    """
    Centers two opposing robots on a random section of
    battlefield. Objective is to inflict damage on
    opponent. Game ends when one dies or when one travels
    a set distance away from opponent.
    Reward - robots will cache a reward proportional to
    damage inflicted on opponent, and the opponent will
    lose this health. Upon leaving radius of conflict, 
    game will terminate and agents will gain cached reward.
    If a robot dies, it will gain no reward.
    """
    def __init__(self):
        """
        This version uses robots 2 and 3 in one-one combat.
        """
        super(CloseQuarter, self).__init__()
        self.rewards = [0, 0]
        self.reward_function = lambda damage: damage

        self.radius = 2
        p,p1 = self.find_init_pose()
        self.gazebo.setModelState('roborts_2',**p)
        self.gazebo.setModelState('roborts_3',**p1)

    def step(self, action1, action2):
        prev_robot_hp = deepcopy(self._robot_hp)
        action1 = [0,0,0,0]+action1[4:]
        action2 = action2[:4]+[0,0,0,0]
        state,_,_,_ = super(CloseQuarter,self).step(action1, action2)
        self.rewards[0] += self.reward_function((self._robot_hp[1][0],prev_robot_hp[1][0]))
        self.rewards[1] += self.reward_function((self._robot_hp[2][0],prev_robot_hp[2][0]))
        distance = L2(self.robot_coords[1], self.robot_coords[2])
        if distance > radius or 0 in self._robot_hp:
            rewards, done = deepcopy(self.rewards), True
        else:
            rewards, done = [0,0], False
        
        return state, rewards, done, _

    def reset(self):
        super(CloseQuarter,self).reset()
        p,p1 = self.find_init_pose()
        self.gazebo.setModelState('roborts_2',**p)
        self.gazebo.setModelState('roborts_3',**p1)
        state = self.get_state()
        self.gazebo.pauseSim()

        return state

    def find_init_pose(self):
        obstacles = self.segments
        robot_radius = L2(.550,.420)/2

        p = {'x':random()*(8.1-robot_radius)+robot_radius,'y':random()*(5.1-robot_radius)+robot_radius}
        while _check_collisions_bounds(obstacles, p, robot_radius):
            p['x'] = random()*(8.1-robot_radius)+robot_radius
            p['y'] = random()*(5.1-robot_radius)+robot_radius
        p['Y'] = random()*math.pi*2

        rad = random()*self.radius
        theta = random()*2*math.pi
        p1 = {'x':p['x']+rad*math.cos(theta),'y':p['y']+rad*math.sin(theta)}
        while _check_collisions_bounds(obstacles, p1, robot_radius):
            rad = random()*self.radius
            theta = random()*2*math.pi
            p1['x'] = p['x']+rad*math.cos(theta)
            p1['y'] = p['y']+rad*math.sin(theta)
        p1['Y'] = random()*math.pi*2
        return p,p1

def _check_collisions_bounds(obstacles, p, rad):
    for obstacle in obstacles:
        if lines_cross(*(obstacle+(p['x']-rad,p['y'],p['x']+rad,p['y'])))\
            or lines_cross(*(obstacle+(p['x'],p['y']-rad,p['x'],p['y']+rad))):
            return True
    if p['x'] < 0 or p['x'] > 8.1 or p['y'] < 0 or p['y'] > 5.1:
        return True
    return False





