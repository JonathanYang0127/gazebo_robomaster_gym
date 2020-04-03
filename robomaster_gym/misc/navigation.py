from .geometry import *
from .utils import *
import networkx as nx

no_op = [0, 0, 0, 0]

cap_speed = robot_max_speed / 2.5
cap_angular_speed = robot_max_angular_speed / 2.5

accel_to_full_speed_time = cap_speed / robot_max_accel
accel_to_full_speed_dis = accel_to_full_speed_time * cap_speed / 2

class Navigator:
    pass

class CriticalPointNavigator(Navigator):

    proximity_threshold = 0.05

    def __init__(self, env):
        self.G = nx.Graph()
        self.env = env
        left_nodes = [(0.500, 0.500), (0.500, 1.650), (0.500, 2.790),
        (1.500, 3.000), (0.800, 3.980), (2.400, 4.000), (3.150, 2.990),
        (3.150, 1.650), (1.900, 1.650), (2.400, 0.500), (4.040, 0.500)]
        right_nodes = [(field_length - x, field_width - y) for x, y in left_nodes]
        self.nodes = left_nodes + right_nodes[::-1]
        self.N = len(self.nodes)

        left_edges = [(0, 1), (1, 2), (1, 3), (1, 8), (2, 3),
        (2, 6), (3, 5), (3, 6), (4, 5), (4, 6), (4, 11), (5, 6), (5, 7),
        (5, 11), (6, 7), (6, 9), (7, 8), (7, 9), (8, 9), (9, 10)]
        right_edges = [(self.N - j - 1, self.N - i - 1) for i, j in left_edges]
        cross_edges = [(6, 14), (7, 15)]
        self.edges = left_edges + right_edges + cross_edges
        self.edges = [(i, j, { 'weight': self.neighbor_cost(self.nodes[i], self.nodes[j]) }) for i, j in self.edges]

        self.G.add_nodes_from(range(self.N))
        self.G.add_edges_from(self.edges)

        self.mover = DummyMover()
        self.prev_goal, self.prev_path = [None] * 4, [None] * 4

    def get_point_at_zone(self, index):
        return [10, 8, 2, 11, 13, 19][index]

    def cost_heuristic(self, pt1, pt2):
        return self.neighbor_cost(pt1, pt2)

    def cost_heuristic(self, pt1, pt2):
        return self.straight_sprint_cost(pt1, pt2)

    def cost(self, pt1, pt2):
        pass

    def neighbor_cost(self, pt1, pt2):
        return distance_tuple(pt1, pt2)

    def get_path(self, pt1, pt2):
        pass

    def get_point(self, i):
        if i < self.N:
            return self.nodes[i]

    def navigate(self, robot_index, dest):
        if not all(self.env._odom_info):
            return
        x, y, yaw = self.env._odom_info[robot_index]
        src = (x, y)
        pt = (round(x, 3), round(y, 3))

        src_pt_ind, min_distance = None, float('inf')
        for _src_ind, _src in enumerate(self.nodes):
            _dis = distance_tuple(src, _src)
            if _dis < min_distance:
                src_pt_ind, min_distance = _src_ind, _dis
                if _dis < self.proximity_threshold:
                    break

        dst_pt_ind, min_distance = None, float('inf')
        for _dst_ind, _dst in enumerate(self.nodes):
            node_x, node_y = _dst
            _dis = distance_tuple(dest, _dst)
            if _dis < min_distance:
                dst_pt_ind, min_distance = _dst_ind, _dis
                if _dis < self.proximity_threshold:
                    break

        if src_pt_ind == dst_pt_ind:
            return self.mover.move_to(self.env._odom_info[robot_index], dest)

        if self.prev_path[robot_index] and dst_pt_ind in self.prev_path[robot_index]:
            path = self.prev_path[robot_index][:self.prev_path[robot_index].index(dst_pt_ind) + 1]
        else:
            path = nx.shortest_path(self.G, src_pt_ind, dst_pt_ind, weight='weight')
        
        if distance_tuple(src, self.nodes[path[0]]) < self.proximity_threshold:
            path = path[1:]
        while len(path) >= 2:
            to_x, to_y = self.nodes[path[1]]
            if self.env.is_line_of_sight_blocked(x, y, to_x, to_y, robot_index, segment_type='move'):
                break
            path = path[1:]

        self.prev_goal[robot_index], self.prev_path[robot_index] = path[-1], path
        if path:
            dest_x, dest_y = self.nodes[path[0]]
            blockade = self.env.is_line_of_sight_blocked(x, y, dest_x, dest_y, robot_index, segment_type='move')
            if not blockade or blockade is True:
                print(path)
                return self.mover.move_to(self.env._odom_info[robot_index], self.nodes[path[0]])

class CriticalPointNavigatorWithRotation(CriticalPointNavigator):

    def __init__(self, env):
        super().__init__(env)
        self.to_angle = None

    def navigate(self, robot_index, dest):
        if not all(self.env._odom_info):
            return
        count = 0
        x, y, yaw = self.env._odom_info[robot_index]
        for enemy_index in self.env.get_enemy_team(robot_index):
            aimed_plate = self.env.best_plates[enemy_index]
            if self.env._num_projectiles[enemy_index] and aimed_plate:
                aimed_index = self. env.best_plates[enemy_index][0]
                if aimed_index == robot_index:

                    enemy_x, enemy_y, _ = self.env._odom_info[enemy_index]
                    if angleDiff(angleTo(x, y, enemy_x, enemy_y), yaw) > math.pi / 10:
                        self.to_angle = angleTo(x, y, enemy_x, enemy_y)
                        count += 1
        if count == 1:
            return self.mover.turn_to_angle(yaw, self.to_angle)
        return super().navigate(robot_index, dest)

class Mover:

    accel = 'accel'
    constant_speed = 'const_sp'
    decel = 'decel'

    cap_speed = robot_max_speed
    cap_angular_speed = robot_max_angular_speed
    cap_accel = robot_max_accel

class DummyMover(Mover):

    cap_speed = robot_max_speed / 1.5
    cap_angular_speed = robot_max_angular_speed / 1.5

    def move_to(self, odom_info, to):
        fr_x, fr_y, yaw = odom_info
        to_x, to_y = to
        return self.move_forward(angleTo(fr_x, fr_y, to_x, to_y) - yaw, self.cap_speed)

    # [x-vel, y-vel, yaw, shoot_flag]
    def move_forward(self, angle, speed):
        x_vel = speed * math.cos(angle)
        y_vel = speed * math.sin(angle)
        return [x_vel, y_vel, 0, 0]

    def turn_to_angle(self, fr, to):
        diff = simplify_angle(to - fr)
        yaw = diff
        if (abs(diff) > self.cap_angular_speed):
            yaw = self.cap_angular_speed if diff > 0 else -self.cap_angular_speed
        return [0, 0, yaw, 0]


class AccelMover(Mover):

    cap_speed = robot_max_speed / 2.5
    cap_angular_speed = robot_max_angular_speed / 2.5

    def __init__(self):
        self.break_distance

        self.prev_speed = 0
        self.prev_phase = None
        self.prev_angular_speed = 0

    def move_forward(self, angle):
        pass

    # def straight_sprint_cost(self, pt1, pt2):
    #     x1, y1 = pt1
    #     x2, y2 = pt2
    #     half_dis = distance(x1, y1, x2, y2) / 2
    #     if half_dis < accel_to_full_speed_dis:
    #         return (half_dis / accel_to_full_speed_dis) ** 0.5 * accel_to_full_speed_time * 2
    #     return ((half_dis - accel_to_full_speed_dis) / robot_max_speed + accel_to_full_speed_time) * 2
