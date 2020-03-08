from robomaster_gym.misc.navigation import *

def doNothing(env, robot_index):
    if not all(env._odom_info):
        return
    x, y, yaw = env._odom_info[robot_index]
    return x, y

class PatrolStrategy:

    def __init__(self, path):
        self.path = path
        self.proximity_threshold = 0.05

    def pick(self, env, robot_index):
        if not all(env._odom_info):
            return
        path = self.path
        x, y, _ = env._odom_info[robot_index]
        to_x, to_y = env.navigator.get_point(path[0])
        if distance(x, y, to_x, to_y) < self.proximity_threshold:
            self.path = path[1:] + [path[0]]
        return self.path[0]


class GetBuffStrategy():

    def pick(self, env, robot_index):
        is_team_01 = robot_index <= 1
        target_zones = [2, 3] if is_team_01 else [-2, -3]
        for target_zone in target_zones:
            _index = env._zone_types.index(target_zone)
            if env._zones_active[_index]:
                return env.navigator.get_point_at_zone(_index)
