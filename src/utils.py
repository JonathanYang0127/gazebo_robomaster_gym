
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
