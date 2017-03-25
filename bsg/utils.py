

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
