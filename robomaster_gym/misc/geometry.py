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
    x, y, z, yaw = odom_info
    coords = []
    for angle in (yaw + robot_diag_angle, yaw + math.pi - robot_diag_angle, yaw + math.pi + robot_diag_angle, yaw - robot_diag_angle):
        xoff, yoff = math.cos(angle) * robot_diag_length, math.sin(angle) * robot_diag_length
        coords.extend([x + xoff, y + yoff])
    return coords


# return the coordinates of armor plates of a robot given its odom info
# format: [(x1, y1, x2, y2)] * 4
def plate_coords_from_odom(odom_info):
    x, y, z, yaw = odom_info
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
        xr, xr, yb, yt = xr - buffer, xr + buffer, yb - buffer, yt + buffer
        segments.extend([(xl, yb, xl, yt), (xl, yb, xr, yb), (xr, yt, xr, yb), (xr, yt, xl, yt)])
    left, hmid, right, bot, vmid, top = center_obstacle
    left, right, bot, top = left - buffer, right + buffer, bot - buffer, top + buffer
    segments.extend([(left, vmid, hmid, top), (hmid, top, right, vmid), (right, vmid, hmid, bot), (hmid, bot, left, vmid)])
    return segments

#################
# LINE OF SIGHT #
#################

def orientation(xp, yp, xq, yq, xr, yr):
    """
    This determines if points p, q, and r are colinear(0,pi),
    clockwise(-1) or counter-clockwise(1) by finding
    magnitude of k-vector in cross-product with p as center
    point. If colinear, this will return the angle <qpr.
    """
    cross = (xq-xp)*(yr-yp) - (xr-xp)*(yq-yp)
    dot = (xq-xp)*(xr-xp) + (yr-yp)*(yq-yp)
    if cross < 0:
        return -1
    elif cross > 0:
        return 1
    elif dot > 0:
        return 0
    else:
        return math.pi

def lines_cross(x1, y1, x2, y2, x3, y3, x4, y4):
    """
    Consider the pair of triangles formed by orientations of
    either point on one line with both points on the other:
    say that the pair is opposing if the orientations are
    different. Then for lines ((x1,y1),(x2,y2) and
    ((x3,y3),(x4,y4)) to cross, either both pairs of triangles
    are opposing, the lines share vertices or one line
    contains the other. Assumes length > 0
    """
    a1 = orientation(x1,y1, x3,y3, x4,y4)
    a2 = orientation(x2,y2, x3,y3, x4,y4)
    b1 = orientation(x3,y3, x1,y1, x2,y2)
    b2 = orientation(x4,y4, x1,y1, x2,y2)
    return ((a1 != a2 and b1 != b2)\
        or ((x1 == x3 and y1 == y3) or (x2 == x3 and y2 == y3)\
            or (x1 == x4 and y1 == y4) or (x2 == x4 and y2 == y4))\
        or ((a1 == math.pi and a2 == math.pi)\
            or b1 == math.pi and b2 == math.pi))

def distance(x1, y1, x2, y2):
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def angleTo(x1, y1, x2, y2):
    assert not (x1 == 0 and y1 == 0) and not (x2 == 0 and y2 == 0), "neither point should be the origin"
    if x1 == x2:
        if y1 < y2:
            return math.pi / 2
        elif y1 == y2:
            return 0
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

if __name__ == "__main__":
    print(orientation(1,1, 2,1, 1,2)) # -1
    print(orientation(1,1, 1,0, 2,1)) # 1
    print(orientation(1,1, 2,1, 0,1)) # pi
    print(orientation(1,1, 2,1, 3,1)) # 0
    print(lines_cross(1,1, 2,1, 3,1, 4,1)) # False
    print(lines_cross(1,1, 3,1, 2,2, 3,2)) # False
    print(lines_cross(1,1, 3,1, 2,0, 3,0)) # False
    print(lines_cross(2,2, 3,2, 1,1, 3,1)) # False
    print(lines_cross(2,0, 3,0, 1,1, 3,1)) # False
    print(lines_cross(1,1, 3,1, 2,2, 2,3)) # False
    print(lines_cross(1,1, 3,1, 2,0, 2,-1)) # False
    print(lines_cross(1,1, 3,1, 2,0, 2,2)) # True
    print(lines_cross(1,1, 3,1, 2,0, 2,1)) # True
    print(lines_cross(1,1, 3,1, 2,1, 2,2)) # True
    print(lines_cross(1,1, 3,1, 2,1, 4,1)) # True
    print(lines_cross(1,1, 3,1, 2,1, 0,1)) # True
    print(lines_cross(1,1, 4,1, 2,1, 3,1)) # True
    print(lines_cross(1,1, 2,1, 2,1, 3,1)) # True
    print(lines_cross(1,1, 2,1, 1,1, 2,1)) # True
