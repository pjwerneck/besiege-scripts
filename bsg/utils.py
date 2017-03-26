

import math
import clr

clr.AddReference("System")
clr.AddReference("UnityEngine")

from UnityEngine import GameObject


def clip(v, minv, maxv):
    return max(minv, min(v, maxv))


def delta_angle(a, b):
    return ((a - b) + 180) % 360 - 180


def mm(v):
    return (-v, +v)


def clear_marks():
    while 1:
        obj = GameObject.Find("Mark")
        try:
            obj.DestroyImmediate(obj)
        except Exception:
            break


def pretty(seq):
    return '(' + ', '.join('%0.2f' % x for x in seq) + ')'


def dsteer(x, y):
    # convert to polar
    r = math.hypot(x, y)
    t = math.atan2(y, x)

    # rotate by 45 degrees
    t += math.pi / 4

    # back to cartesian
    left = r * math.cos(t)
    right = r * math.sin(t)

    # rescale the new coords
    left = left * math.sqrt(2)
    right = right * math.sqrt(2)

    # clamp to -1/+1
    left = max(-1, min(left, 1))
    right = max(-1, min(right, 1))

    return left, right
