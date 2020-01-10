from robomaster_env import *
from copy import deepcopy
import scipy.spatial.distance.euclidean as L2
from numpy.random import random
from utils import *

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
        self._robot_hp - [r1_hp, r2_hp, r3_hp, r4_hp]
        self._num_projectiles = [[50], [50], [50], [50]]
        self._barrel_heat = [[0], [0], [0], [0]]
        self._shoot -
        self._robot_effects
        """
        super.__init__()
        self.rewards = [0, 0]
        self.reward_function = lambda damage: damage

        self.radius = 2
        init_pose = self.find_init_pose()

        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        cli_args1 = ['gazebo_robomaster_gym','custom_loc_multi_robot.launch',\
            'x2:=%d'.format(init_pose[0]),'y2:=%d'.format(init_pose[1]),'Y2:=%d'.format(init_pose[2]),\
            'x3:=%d'.format(init_pose[3]),'y3:=%d'.format(init_pose[4]),'Y3:=%d'.format(init_pose[5]),]
        cli_args2 = ['roborts_bringup','roborts_gazebo.launch','gui:=true']
        roslaunch_file1 = roslaunch.rlutil.resolve_launch_arguments(cli_args1)
        roslaunch_args1 = cli_args1[2:]
        roslaunch_file2 = roslaunch.rlutil.resolve_launch_arguments(cli_args2)
        roslaunch_args2 = cli_args2[2:]
        launch_files = [(roslaunch_file1,roslaunch_args1),(roslaunch_file2,roslaunch_args2)]
        self.launch = roslaunch.parent.ROSLaunchParent(uuid, launch_files)
        self.launch.start()

    def step(self, action1, action2):
        prev_robot_hp = deepcopy(self._robot_hp)
        action1 = [0,0,0,0]+action1[4:]
        action2 = action2[:4]+[0,0,0,0]
        state,_,_,_ = super.step(action1, action2)
        self.rewards[0] += self.reward_function((self._robot_hp[1][0],prev_robot_hp[1][0]))
        self.rewards[1] += self.reward_function((self._robot_hp[2][0],prev_robot_hp[2][0]))
        distance = L2(self.robot_coords[0], self.robot_coords[1])
        if distance > radius or 0 in self._robot_hp:
            rewards, done = deepcopy(self.rewards), True
        else:
            rewards, done = [0,0], False
        
        return state, rewards, done, _

    def find_init_pose(self):
        obstacles = self.segments+[[0,0, 0,5.1],[0,5.1, 8.1,5.1],[8.1,5.1, 8.1,0],[8.1,0, 0,0]]
        robot_radius = L2(.550,.420)/2

        p = [random()*(8.1-robot_radius)+robot_radius,random()*(5.1-robot_radius)+robot_radius]
        while _check_collisions_bounds(obstacles, p, robot_radius):
            p = [random()*(8.1-robot_radius)+robot_radius,random()*(5.1-robot_radius)+robot_radius]
        init_pose = p
        init_pose.append(random()*2*math.pi)

        rad = random()*self.radius
        theta = random()*2*math.pi
        p1 = [p[0]+rad*math.cos(theta),p[1]+rad*math.sin(theta)]
        while _check_collisions_bounds(obstacles, p1, robot_radius):
            rad = random()*self.radius
            theta = random()*2*math.pi
            p1 = [p[0]+rad*math.cos(theta),p[1]+rad*math.sin(theta)]
        init_pose += p1
        init_pose.append(random()*2*math.pi)
        return init_pose

def _check_collisions_bounds(obstacles, p, rad):
    for obstacle in obstacles:
        if lines_cross(*(obstacle+[p[0]-rad,p[1],p[0]+rad,p[1]])) or lines_cross(*(obstacle+[p[0],p[1]-rad,p[0],p[1]+rad])):
            return True
    if p[0] < 0 or p[0] > 8.1 or p[1] < 0 or p[1] > 5.1:
        return True
    return False





