

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


def mean(data):
    """Return the sample arithmetic mean of data."""
    n = len(data)
    if n < 1:
        raise ValueError('mean requires at least one data point')

    return sum(data) / n  # in Python 2 use sum(data)/float(n)


def _ss(data):
    """Return sum of square deviations of sequence data."""
    c = mean(data)
    ss = sum((x - c) ** 2 for x in data)
    return ss


def pstdev(data):
    """Calculates the population standard deviation."""
    n = len(data)
    if n < 2:
        raise ValueError('variance requires at least two data points')
    ss = _ss(data)
    pvar = ss / n  # the population variance
    return pvar ** 0.5
