
import math

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

# angle 2-1-3, guaranteed to be on the side <= pi
def angleBetween(x1, y1, x2, y2, x3, y3):
    return angleDiff(angleTo(x1, y1, x2, y2), angleTo(x1, y1, x3, y3))

def angleDiff(a1, a2):
    diff = abs(a1 - a2)
    return min(diff, math.pi * 2 - diff)

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

