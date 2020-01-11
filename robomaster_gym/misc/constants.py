import math

field_length = 8.08
field_width = 4.48

robot_length = 0.550
robot_width = 0.420
armor_size = 0.131
armor_thickness = 0.015

gimbal_range_deg = 82.5
gimbal_range_angle = gimbal_range_deg / 180 * math.pi

# TODO: determine by experiment
gimbal_range_dis = float('inf')

robot_max_hp = 2000
max_launch_velocity = 25

armor_plate_damage = [20, 40, 40, 60]

# x-start, x-end, y-start, y-end
parallel_obstacles = [
    (1.500, 1.750, 0, 1.000),
    (3.600, 4.600, 1.000, 1.250),
    (7.100, 8.100, 1.000, 1.250),
    (1.500, 2.300, 2.425, 2.675),
    (5.800, 6.600, 2.425, 2.675),
    (0.000, 1.000, 3.850, 4.100),
    (3.500, 4.500, 3.850, 4.100),
    (6.350, 6.600, 4.100, 5.100),
]

# [left, middle, right, bottom, middle, top]
center_x, center_y = 4.050, 2.550
diagonal_len = 0.300 / math.sqrt(2)
center_obstacle = (center_x - diagonal_len, center_x, center_x + diagonal_len, center_y - diagonal_len, center_y, center_y + diagonal_len)

zones = [
    (3.830, 4.370, 0.270, 0.750),
    (1.630, 2.170, 1.695, 2.175),
    (0.230, 0.770, 3.120, 3.600),
    (4.350, 4.830, 4.500, 5.040),
    (5.330, 7.870, 1.500, 1.980),
    (5.930, 6.470, 2.925, 3.405)]
