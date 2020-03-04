from robomaster_gym.misc.navigation import *

def doNothing(env, robot_index):
    x, y, yaw = env._odom_info[robot_index]
    return x, y
