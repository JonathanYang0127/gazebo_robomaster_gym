from .geometry import *
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
        (1.500, 3.000), (0.500, 3.980), (2.400, 4.000), (3.150, 2.990),
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

        self.G.add_nodes_from(self.nodes[1:])
        self.G.add_edges_from(self.edges)

        self.mover = DummyMover()

    def cost_heuristic(self, pt1, pt2):
        return self.neighbor_cost(pt1, pt2)

    def cost(self, pt1, pt2):
        pass

    def neighbor_cost(self, pt1, pt2):
        return distance(*pt1, *pt2)

    def get_path(self, pt1, pt2):
        pass

    def get_point(self, i):
        if i < self.N:
            return self.nodes[i]

    def navigate(self, robot_index, dest):
        if not all(self.env._odom_info):
            return
        x, y, yaw = self.env._odom_info[robot_index]
        pt = (round(x, 3), round(y, 3))

        src_pt_ind, min_distance = None, float('inf')
        for _src_ind, _src in enumerate(self.nodes):
            node_x, node_y = _src
            _dis = distance(x, y, node_x, node_y)
            if _dis < min_distance:
                src_pt_ind, min_distance = _src_ind, _dis
                if _dis < self.proximity_threshold:
                    break

        d_x, d_y = dest
        dst_pt_ind, min_distance = None, float('inf')
        for _dst_ind, _dst in enumerate(self.nodes):
            node_x, node_y = _dst
            _dis = distance(d_x, d_y, node_x, node_y)
            if _dis < min_distance:
                dst_pt_ind, min_distance = _dst_ind, _dis
                if _dis < self.proximity_threshold:
                    break

        if src_pt_ind == dst_pt_ind:
            return self.mover.move_to(pt, dest)

        path = nx.shortest_path(self.G, src_pt_ind, dst_pt_ind)
        index = 0
        while index < len(path):
            to_x, to_y = self.nodes[path[index]]
            if self.env.is_line_of_sight_blocked(x, y, to_x, to_y, robot_index):
                break
            index += 1

        if index == len(path):
            index = -1
        print(index)
        return self.mover.move_to(pt, self.nodes[path[index]])

class Mover:

    accel = 'accel'
    constant_speed = 'const_sp'
    decel = 'decel'

    cap_speed = robot_max_speed
    cap_angular_speed = robot_max_angular_speed
    cap_accel = robot_max_accel

class DummyMover(Mover):

    cap_speed = robot_max_speed / 2.5
    cap_angular_speed = robot_max_angular_speed / 2.5

    def move_to(self, fr, to):
        fr_x, fr_y = fr
        to_x, to_y = to
        return self.move_forward(angleTo(fr_x, fr_y, to_x, to_y), self.cap_speed)

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
