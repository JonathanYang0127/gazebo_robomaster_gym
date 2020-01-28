import math

field_length = 8.08
field_width = 4.48

robot_length = 0.550
robot_width = 0.420
armor_size = 0.131
armor_thickness = 0.015

robot_max_speed = 3.11
robot_max_angular_speed_deg = 330
robot_max_angular_speed = robot_max_angular_speed_deg / 180 * math.pi
robot_max_accel = 0.7

gimbal_range_deg = 82.5
gimbal_range_angle = gimbal_range_deg / 180 * math.pi

# TODO: determine by experiment
gimbal_range_dis = float('inf')

robot_max_hp = 2000
max_launch_velocity = 25

armor_plate_damage = [20, 40, 40, 60]

def symmetric_rectangle(tup):
    l, r, b, t = tup
    return (round(field_length - r, 3), round(field_length - l, 3), round(field_width - t, 3), round(field_width - b, 3))

# x-start, x-end, y-start, y-end
parallel_obstacles_bottom = [
    (1.500, 1.700, 0.000, 1.000),
    (3.540, 4.540, 0.935, 1.135),
    (7.080, 8.080, 1.000, 1.200),
    (1.500, 2.300, 2.140, 2.340),
]
parallel_obstacles = parallel_obstacles_bottom + [symmetric_rectangle(rec) for rec in parallel_obstacles_bottom]

# [left, middle, right, bottom, middle, top]
center_x, center_y = field_length / 2, field_width / 2
diagonal_len = 0.250 / math.sqrt(2)
center_obstacle = (center_x - diagonal_len, center_x, center_x + diagonal_len, \
    center_y - diagonal_len, center_y, center_y + diagonal_len)

zones_left = [
    (3.770, 4.310, 0.205, 0.685),
    (1.630, 2.170, 1.410, 1.890),
    (0.230, 0.770, 2.550, 3.030),
]
zones = zones_left + [symmetric_rectangle(rec) for rec in zones_left]

float_eq_threshold = 1e-05
