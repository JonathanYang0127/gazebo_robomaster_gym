from .geometry import *
import networkx as nx

no_op = [0, 0, 0, 0]

accel_to_full_speed_time = robot_max_speed / robot_max_accel
accel_to_full_speed_dis = accel_to_full_speed_time * robot_max_speed / 2

class Navigator:
    def straight_sprint_cost(self, pt1, pt2):
        x1, y1 = pt1
        x2, y2 = pt2
        half_dis = distance(x1, y1, x2, y2) / 2
        if half_dis < accel_to_full_speed_dis:
            return (half_dis / accel_to_full_speed_dis) ** 0.5 * accel_to_full_speed_time * 2
        return ((half_dis - accel_to_full_speed_dis) / robot_max_speed + accel_to_full_speed_time) * 2

class CriticalPointNavigator(Navigator):

    def __init__(self, env):
        self.G = nx.Graph()
        self.points = []

    def cost(self, pt1, pt2):
        pass
        # return self.straight_sprint_cost(pt1, pt2)

    def get_path(self, pt1, pt2):
        pass

    def navigate(self, robot_index, dest):
        pass
