from .constants import *

#####################
# ROBOT COORDINATES #
#####################

# calculate diagonal angle and length of a rectangle
# l: length
# w: width
def diag_stats_helper(l, w):
    angle = math.atan(w / l)
    return angle, w / math.sin(angle) / 2

forward_armor_frame_length = robot_length + armor_thickness * 2
sideway_armor_frame_width = robot_width + armor_thickness * 2

robot_diag_angle, robot_diag_length = diag_stats_helper(robot_length, robot_width)
forward_armor_diag_angle, forward_armor_diag_length = diag_stats_helper(forward_armor_frame_length, armor_size)
sideway_armor_diag_angle, sideway_armor_diag_length = diag_stats_helper(armor_size, sideway_armor_frame_width)


def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    X = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    Z = math.atan2(t3, t4)

    return X, Y, Z


# return the coordinates of corners of a robot given its odom info
# format: [(x1, y1, x2, y2)] * 4
def robot_coords_from_odom(odom_info):
    x, y, yaw = odom_info
    coords = []
    for angle in (yaw + robot_diag_angle, yaw + math.pi - robot_diag_angle, yaw + math.pi + robot_diag_angle, yaw - robot_diag_angle):
        xoff, yoff = math.cos(angle) * robot_diag_length, math.sin(angle) * robot_diag_length
        coords.extend([x + xoff, y + yoff])
    return coords


# return the coordinates of armor plates of a robot given its odom info
# format: [(x1, y1, x2, y2)] * 4
def plate_coords_from_odom(odom_info):
    x, y, yaw = odom_info
    frontx1off, fronty1off = math.cos(yaw + forward_armor_diag_angle) * forward_armor_diag_length, math.sin(yaw + forward_armor_diag_angle) * forward_armor_diag_length
    frontx2off, fronty2off = math.cos(yaw - forward_armor_diag_angle) * forward_armor_diag_length, math.sin(yaw - forward_armor_diag_angle) * forward_armor_diag_length
    backx1off, backy1off = math.cos(yaw + math.pi + forward_armor_diag_angle) * forward_armor_diag_length, math.sin(yaw + math.pi + forward_armor_diag_angle) * forward_armor_diag_length
    backx2off, backy2off = math.cos(yaw + math.pi - forward_armor_diag_angle) * forward_armor_diag_length, math.sin(yaw + math.pi - forward_armor_diag_angle) * forward_armor_diag_length
    leftx1off, lefty1off = math.cos(yaw + sideway_armor_diag_angle) * sideway_armor_diag_length, math.sin(yaw + math.pi - sideway_armor_diag_angle) * sideway_armor_diag_length
    rightx2off, righty2off  = math.cos(yaw - sideway_armor_diag_angle) * sideway_armor_diag_length, math.sin(yaw - sideway_armor_diag_angle) * sideway_armor_diag_length
    rightx1off, righty1off = math.cos(yaw + math.pi + sideway_armor_diag_angle) * sideway_armor_diag_length, math.sin(yaw + math.pi + sideway_armor_diag_angle) * sideway_armor_diag_length
    leftx2off, lefty2off = math.cos(yaw + math.pi - sideway_armor_diag_angle) * sideway_armor_diag_length, math.sin(yaw + math.pi - sideway_armor_diag_angle) * sideway_armor_diag_length
    return [(x + frontx1off, y + fronty1off, x + frontx2off, y + fronty2off),
            (x + leftx1off, y + lefty1off, x + leftx2off, y + lefty2off),
            (x + rightx1off, y + righty1off, x + rightx2off, y + righty2off),
            (x + backx1off, y + backy1off, x + backx2off, y + backy2off)]


#####################
# ZONE AND OBSTACLE #
#####################

def generate_obstacle_segments(buffer):
    segments = []
    for xl, xr, yb, yt in parallel_obstacles:
        xl, xr, yb, yt = xl - buffer, xr + buffer, yb - buffer, yt + buffer
        segments.extend([(xl, yb, xl, yt), (xl, yb, xr, yb), (xr, yt, xr, yb), (xr, yt, xl, yt)])
    left, vmid, right, bot, hmid, top = center_obstacle
    return segments + [(left - buffer, hmid, right + buffer, hmid), (vmid, bot - buffer, vmid, top + buffer)]

#################
# LINE OF SIGHT #
#################

def counterClockwise(ax, ay, bx, by, cx, cy):
    return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

def lines_cross(x1, y1, x2, y2, x3, y3, x4, y4):
    if x3 == x4:
        if x1 == x2 or (x1 - x3) * (x2 - x3) > 0:
            return False # colinear counts as no cross due to buffer
        y = (x3 - x1) / (x2 - x1) * (y2 - y1) + y1
        return (y3 - y) * (y4 - y) < 0
    elif y3 == y4:
        if y1 == y2 or (y1 - y3) * (y2 - y3) > 0:
            return False
        x = (y3 - y1) / (y2 - y1) * (x2 - x1) + x1
        return (x3 - x) * (x4 - x) < 0
    else:
        # general case; we still ignore colinear cases
        return counterClockwise(x1, y1, x3, y3, x4, y4) != counterClockwise(x2, y2, x3, y3, x4, y4) and \
            counterClockwise(x1, y1, x2, y2, x3, y3) != counterClockwise(x1, y1, x2, y2, x4, y4)

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

# between 0 and 2*pi
def angleTo(x1, y1, x2, y2):
    if x1 == x2:
        if y1 < y2:
            return math.pi / 2
        return math.pi * 3 / 2
    dx, dy = x2 - x1, y2 - y1
    rawDeg = math.atan(dy / dx)
    if dx < 0:
        rawDeg += math.pi
    return rawDeg % (math.pi * 2)

# angle between (1 - 2) and (3 - 4), guaranteed to be on the side <= pi
def angleBetween(x1, y1, x2, y2, x3, y3, x4, y4):
    return angleDiff(angleTo(x1, y1, x2, y2), angleTo(x3, y3, x4, y4))

def angleDiff(a1, a2):
    diff = abs(a1 - a2)
    return min(diff, math.pi * 2 - diff)

def midpoint(x1, y1, x2, y2):
    return (x1 + x2) / 2, (y1 + y2) / 2

def simplify_angle(angle):
    """
    avoid large turning (turning more than 180 degrees)
    :param angle: current angle in RAD
    """
    if angle > math.pi / 2:
        return math.pi / 2 - angle
    return angle