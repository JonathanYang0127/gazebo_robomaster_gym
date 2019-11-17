
import math

def counterClockwise(ax, ay, bx, by, cx, cy):
    return (cy - ay) * (bx - ax) > (by - ay) * (cx - ax)

# assumes (x3, y3) <-> (x4, y4) is either parallel or perpendicular to X-axis
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

# angle 2-1-3, guaranteed to be on the side <= pi
def angleBetween(x1, y1, x2, y2, x3, y3):
    return angleDiff(angleTo(x1, y1, x2, y2), angleTo(x1, y1, x3, y3))

def angleDiff(a1, a2):
    diff = abs(a1 - a2)
    return min(diff, math.pi * 2 - diff)
