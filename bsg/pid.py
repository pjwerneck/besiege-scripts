

import math

from .utils import clip



class PID(object):
    """PID controller

    """

    __slots__ = ['Kp', 'Ki', 'Kd', 'setpoint', 'last_error', 'integral', 'limit', 'limit_i',
                 'gain_f']

    def __init__(self, gain, limit=None, limit_i=None):
        self.Kp, self.Ki, self.Kd = gain

        self.setpoint = 0

        self.last_error = 0
        self.integral = 0

        self.limit = limit
        self.limit_i = limit_i

        self.gain_f = 1

    def _sub(self, a, b):
        return a - b

    def update(self, value, dt):
        error = self._sub(self.setpoint, value)

        self.integral = self.integral + error * dt * self.Ki * self.gain_f
        if self.limit_i:
            self.integral = clip(self.integral, *self.limit_i)

        d = (error - self.last_error) / dt

        self.last_error = error

        out = (self.Kp * error * self.gain_f) + self.integral + (self.Kd * d * self.gain_f)

        if self.limit:
            out = clip(out, *self.limit)

        return out

    def reset(self):
        self.integral = 0
        self.last_error = 0
        self.gain_f = 1


class AngularPID(PID):
    """PID controller for angular values. It uses the shortest distance
    between two angles for the error.

    """

    def _sub(self, a, b):
        return ((a - b) + 180) % 360 - 180
