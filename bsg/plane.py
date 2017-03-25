

import clr

clr.AddReference("System")
clr.AddReference("UnityEngine")

from UnityEngine import Input
from UnityEngine import Physics
from UnityEngine import Quaternion
from UnityEngine import Time
from UnityEngine import Vector2
from UnityEngine import Vector3
from UnityEngine import Vector4
from UnityEngine import Color

clr.AddReference("LenchScripterMod")
from Lench.Scripter import Functions as Besiege

from . import pid
from .utils import mm
from .utils import clear_marks


class Waypoint(object):
    __slots__ = ['position', 'mark']

    def __init__(self, position, mark=None):
        self.position = position
        self.mark = mark or Besiege.CreateMark(position)

    def clear(self):
        if self.mark:
            self.mark.Clear()


class BasePlane(object):

    def __init__(self, **config):
        clear_marks()

        self.core = config['core_block']
        self.core_height = None

        self.collision_radius = (config['collision_min_radius'],
                                 config['collision_med_radius'],
                                 config['collision_max_radius'])

        self.collision_points = (Vector3(), Vector3(), Vector3())

        self.motors = {k: v for (k, v) in config.iteritems() if k.startswith('motor_')}

        self.axes = {'throttle': config['axis_throttle'],
                     'pitch': config['axis_pitch'],
                     'yaw': config['axis_yaw'],
                     'roll': config['axis_roll'],
                     }

        self.keys = {k.split('_', 1)[1]: v for (k, v) in config.iteritems() if k.startswith('key_')}

        self.mode = self.rate_mode

        self.position = Vector3()
        self.position_sp = Vector3()

        self.rotation = Vector3()
        self.rotation_sp = Vector3()

        self.rate_sp = Vector3()
        self.velocity_sp = Vector3()

        self.rate_pid = (
            pid.PID(config['rate_gain_pitch']),
            pid.PID(config['rate_gain_yaw']),
            pid.PID(config['rate_gain_roll']),
            )

        self.stabilize_pid = (
            pid.AngularPID(config['stabilize_gain_pitch'], limit_i=(-1, 1)),
            pid.AngularPID(config['stabilize_gain_yaw'], limit_i=(-1, 1)),
            pid.AngularPID(config['stabilize_gain_roll'], limit_i=(-1, 1))
            )

        self.velocity_pid = (
            pid.PID(config['velocity_gain_x'], limit=mm(1), limit_i=mm(config['velocity_cf_x'])),
            pid.PID(config['velocity_gain_y'], limit=mm(1), limit_i=mm(config['velocity_cf_y'])),
            pid.PID(config['velocity_gain_z'], limit=mm(1), limit_i=mm(config['velocity_cf_z'])),
            )

        self.althold_pid = pid.PID(config['althold_gain'], limit=mm(1))

        self.poshold_pid = (
            pid.PID(config['poshold_gain_x'], limit=mm(1)),
            None,
            pid.PID(config['poshold_gain_z'], limit=(-1, 1)),
            )

        self.auto_pid = (
            pid.PID(config['auto_gain_x'], limit=(-1, 1)),
            pid.PID(config['auto_gain_y'], limit=(-1, 1)),
            pid.PID(config['auto_gain_z'], limit=(-1, 1)),
            )

        self.plane_mode_pid = pid.PID(config['plane_gain'], limit=(-1, 1))

        self.home = None

        self._hold_yaw = None

        self.colmarks = [None, None, None]

        clear_marks()

        self.waypoints = []

        self.waypoints = [Waypoint(Vector3(-252.1, 76, 78.4)),
                          Waypoint(Vector3(-353.5, 123.4, 405.6)),
                          Waypoint(Vector3(233.6, 269.0, 84.7)),
                          Waypoint(Vector3(176.9, 243.2, 314.7)),
                          Waypoint(Vector3(0, 2, 0))]

        self.current_waypoint = None
        self.config = config

        self._first_update = True


    def get_controls(self):
        return [round(self.axes[k].OutputValue, 2) for k in ('throttle', 'pitch', 'yaw', 'roll')]

    def update_keys(self):
        for name, keys in self.keys.iteritems():
            for k in keys:
                if Input.GetKeyDown(k):
                    getattr(self, name)()
                    break

    def update(self):
        self.dt = Time.deltaTime

        if self.dt == 0:
            return

        if self._first_update:
            self.home = self.core.Position

            if self.core_height is None:
                self.core_height = self.core.Position.y
            # Besiege.ClearMarks()  # ClearMarks is broken

            self._first_update = False
            return

        # check for key press events and respective callbacks
        self.update_keys()

        # get the current relative angular velocity
        self.rate = Quaternion.Inverse(self.core.RotationQuaternion) * self.core.AngularVelocityDeg

        # get the current relative angles
        self.rotation = self.core.Rotation

        # get the current world position
        self.position = self.core.Position

        # get the current axes values
        self.controls = self.get_controls()

        # A mode function can return None if it's delegating to
        # another mode. In that case, we loop until we get a power
        # adjustment response
        power = None
        while power is None:
            power = self.mode(*self.controls)

        Besiege.Watch('dt', self.dt)
        Besiege.Watch('mode', self.mode.__name__)

        # using Vector4 merely for the nice string repr
        Besiege.Watch('controls', Vector4(*self.controls))

        Besiege.Watch('rotation', self.rotation)
        Besiege.Watch('rotation_sp', self.rotation_sp)

        Besiege.Watch('position', self.position)
        Besiege.Watch('position_sp', self.position_sp)
        Besiege.Watch('position_sp_local', self.position_sp_local)
        Besiege.Watch('position_sp_distance', self.position_sp_distance)

        Besiege.Watch('rate', self.rate)
        Besiege.Watch('rate_sp', self.rate_sp)

        Besiege.Watch('velocity', self.velocity)
        Besiege.Watch('velocity_sp', self.velocity_sp)

        # get ASL and real altitude
        Besiege.Watch('altitude', Vector2(self.position.y, self.position.y - self.terrain))

        self.set_power(*power)

    def set_rate_mode(self):
        self.mode = self.rate_mode

    def rate_mode(self, throttle, pitch, yaw, roll):
        """In Rate Mode, pitch, yaw and roll axes control the rate of rotation
        in the respective axes. The throttle axis controls motor power
        directly.

        """
        self.rate_sp.Set(
            pitch * self.config['rate_cf_pitch'],
            yaw * self.config['rate_cf_yaw'],
            roll * self.config['rate_cf_roll'],
            )

        self.rate_pid[0].setpoint = self.rate_sp[0]
        self.rate_pid[1].setpoint = self.rate_sp[1]
        self.rate_pid[2].setpoint = self.rate_sp[2]

        pitch_adj = self.rate_pid[0].update(self.rate[0], self.dt)
        yaw_adj = self.rate_pid[1].update(self.rate[1], self.dt)
        roll_adj = self.rate_pid[2].update(self.rate[2], self.dt)

        # scale throttle
        throttle_adj = throttle * self.config['rate_cf_throttle']

        # the mainloop will call set_power() with these values
        return throttle_adj, pitch_adj, yaw_adj, roll_adj

    def set_stabilize_mode(self):
        self.mode = self.stabilize_mode

    def stabilize_mode(self, throttle, pitch, yaw, roll):
        """In stabilize mode, the pitch and roll axes control the current
        angle in the respective axes, with the vehicle self-levelling
        when they are released. The yaw axis works as if in rate mode,
        and holds the current yaw when released. Throttle works as if
        in rate mode.

        """

        maxp = self.config['stabilize_max_pitch']
        maxr = self.config['stabilize_max_roll']

        pitch = pid.clip(pitch * self.config['stabilize_cf_pitch'], -maxp, maxp)
        roll = pid.clip(roll * self.config['stabilize_cf_roll'], -maxr, maxr)

        self.rotation_sp.Set(pitch, self.rotation_sp[1], roll)

        self.stabilize_pid[0].setpoint = self.rotation_sp[0]
        self.stabilize_pid[2].setpoint = self.rotation_sp[2]

        pitch_adj = self.stabilize_pid[0].update(self.rotation[0], self.dt)
        roll_adj = self.stabilize_pid[2].update(self.rotation[2], self.dt)

        # yaw input controls the rate as if in rate mode, but if
        # the yaw stick is released, the quad holds the current yaw
        if yaw:
            yaw_adj = yaw
            self._hold_yaw = None

        else:
            if self._hold_yaw is None:
                self._hold_yaw = self.rotation[1]
                self.rotation_sp[1] = self.rotation[1]
                self.stabilize_pid[1].reset()

            self.stabilize_pid[1].setpoint = self.rotation_sp[1]
            yaw_adj = self.stabilize_pid[1].update(self.rotation[1], self.dt)

        self.rotation_sp = Vector3(self.stabilize_pid[0].setpoint,
                                   self.stabilize_pid[1].setpoint,
                                   self.stabilize_pid[2].setpoint)

        return self.rate_mode(throttle, pitch_adj, yaw_adj, roll_adj)

    def set_velocity_mode(self):
        self.mode = self.velocity_mode

        self.position_sp[1] = self.position[1]
        self.althold_pid.reset()

        self.velocity_pid[0].reset()
        self.velocity_pid[1].reset()
        self.velocity_pid[2].reset()

    def velocity_mode(self, throttle, pitch, yaw, roll):
        """In Velocity mode, the roll, throttle and pitch axes control
        velocity in the x, y, z axes, respectively, relative to the
        vehicle's current heading. Yaw works as if in the stabilize
        mode.

        Piloting in the velocity mode is not recommended. This mode is
        used only by the other autonomous modes.

        """
        # scale the controls to m/s
        self.velocity_sp.Set(-roll * self.config['velocity_cf_x'],
                             throttle * self.config['velocity_cf_y'],
                             pitch * self.config['velocity_cf_z'],
                             )

        self.velocity_pid[0].setpoint = self.velocity_sp[0]
        self.velocity_pid[1].setpoint = self.velocity_sp[1]
        self.velocity_pid[2].setpoint = self.velocity_sp[2]

        pitch_adj = self.velocity_pid[2].update(self.velocity[2], self.dt)
        roll_adj = -self.velocity_pid[0].update(self.velocity[0], self.dt)

        throttle_adj = self.velocity_pid[1].update(self.velocity[1], self.dt)

        return self.stabilize_mode(throttle_adj, pitch_adj, yaw, roll_adj)

    def set_althold_mode(self):
        self._hold_yaw = None
        self.mode = self.althold_mode

        self.position_sp[1] = self.position[1]
        self.althold_pid.reset()

    def althold_mode(self, throttle, pitch, yaw, roll):
        """In Altitude Hold mode, the vehicle acts as if in stabilize mode,
        but trying to maintain the current altitude. The throttle axis
        changes the target altitude.

        """

        if throttle:
            mod_rate = throttle * self.config['althold_cf'] * self.dt
            # don't go below zero
            self.position_sp.Set(self.position_sp[0],
                                 max(self.position_sp[1] + mod_rate, 0),
                                 self.position_sp[2])

        throttle_adj = self.althold_pid.update(-self.position_sp_local[1], self.dt)

        return self.velocity_mode(throttle_adj, pitch, yaw, roll)

    def set_poshold_mode(self):
        self._hold_yaw = None
        self.mode = self.poshold_mode

        self.position_sp.Set(self.position[0], self.position[1], self.position[2])

        self.poshold_pid[0].reset()
        self.poshold_pid[2].reset()

        self.althold_pid.reset()

    def poshold_mode(self, throttle, pitch, yaw, roll):
        """In Position Hold mode, the vehicle holds the current horizontal
        position and altitude. Throttle adjusts altitude as if in
        Altitude Hold mode. Pitch and roll adjust the horizontal
        position. Yaw works as if in stabilize mode.

        """
        if roll or pitch:
            delta = Vector3(-roll * self.dt * self.config['poshold_cf_x'],
                            0,
                            pitch * self.dt * self.config['poshold_cf_z'],
                            )

            delta = self.quat_ry * delta

            self.position_sp += delta

        pitch_adj = self.poshold_pid[2].update(-self.position_sp_local[2], self.dt)
        roll_adj = self.poshold_pid[0].update(self.position_sp_local[0], self.dt)

        return self.althold_mode(throttle, pitch_adj, yaw, roll_adj)

    def set_rth_mode(self):
        self.mode = self.rth_mode

        self.position_sp.Set(self.home[0],
                             max(self.position.y, self.config['rth_min_y']),
                             self.home[2])

    def rth_mode(self, throttle, pitch, yaw, roll):
        """In RTH (Return to Home) mode, the vehicle autopilots back to the
        initial horizontal position and engages Land mode. Controls
        are ignored.

        """
        if self.position_sp_distance[2] > 1:
            return self.goto_position()

        self.set_land_mode()

    def set_land_mode(self):
        self.mode = self.land_mode

        self.position_sp.Set(self.position[0],
                             self.position[1],
                             self.position[2])

        self.stabilize_pid[0].reset()
        self.stabilize_pid[1].reset()
        self.stabilize_pid[2].reset()

        self._hold_yaw = None

    def land_mode(self, throttle, pitch, yaw, roll):
        """In Land mode, the vehicle will enter Position hold mode until it
        stabilizes, then descend and land on the current position.

        """
        # wait for the vehicle to stabilize at max 5 deg/s rate
        if sum(map(abs, self.rate)) > 5:
            return self.poshold_mode(0, 0, 0, 0)

        self.position_sp.Set(self.position_sp[0],
                             self.terrain + self.core_height,
                             self.position_sp[2])

        return self.poshold_mode(0, 0, 0, 0)

    def set_auto_mode(self):
        self._hold_yaw = None
        self.mode = self.auto_mode

    def auto_mode(self, throttle, pitch, yaw, roll):
        """In Auto mode, the vehicle will move autonomously to every
        waypoint, engaging Position hold mode on the last one.

        """
        if self.current_waypoint:
            if self.position_sp_distance[2] > self.config['auto_radius']:
                return self.goto_position()

            else:
                self.current_waypoint.clear()
                self.current_waypoint = None

        if self.current_waypoint is None:
            if self.waypoints:
                self.current_waypoint = self.waypoints.pop(0)
                self.position_sp = self.current_waypoint.position
                return self.poshold_mode(0, 0, 0, 0)

            else:
                self.set_poshold_mode()

    def goto_position(self):
        h, v, t = self.position_sp_distance

        x, y, z = self.position_sp_local

        self.auto_pid[0].gain_f = 0.8
        self.auto_pid[1].gain_f = 0.8
        self.auto_pid[2].gain_f = 0.8

        # TODO: have to find a better p value that gives a smooth
        # ascent and descent curve.
        p = 0.2

        # if below target, prioritize climbing, so lower xz gain
        if h > 10 and v > 10:
            self.auto_pid[0].gain_f = p
            self.auto_pid[2].gain_f = p

        # if above target, prioritize moving, so lower y gain
        if h > 10 and v < -10:
            self.auto_pid[1].gain_f = p

        #Besiege.Watch('p', p)

        roll_adj = self.auto_pid[0].update(self.position_sp_local[0], self.dt)
        throttle_adj = self.auto_pid[1].update(-self.position_sp_local[1], self.dt)
        pitch_adj = self.auto_pid[2].update(-self.position_sp_local[2], self.dt)

        if self.yaw_mode == 'lock':
            yaw_adj = 0

        elif self.yaw_mode == 'heading':
            yaw_adj = -self.plane_mode_pid.update(self.velocity.x, self.dt)

        return self.velocity_mode(throttle_adj, pitch_adj, yaw_adj, roll_adj)

    def set_plane_mode(self):
        self.mode = self.plane_mode
        self.plane_mode_pid.reset()

    def plane_mode(self, throttle, pitch, yaw, roll):
        """The Plane mode is exactly like rate mode, but the vehicle can be
        controlled using only the pitch and roll axes. It will
        automatically adjust yaw to zero all lateral velocity.

        """
        if self.velocity.z > self.config['plane_min_z']:
            yaw = -self.plane_mode_pid.update(self.velocity.x, self.dt)

        return self.rate_mode(throttle, pitch, yaw, roll)

    def set_brake_mode(self):
        self.mode = self.brake_mode

        self.position_sp[1] = self.position[1]
        self.althold_pid.reset()

        self._hold_yaw = None

    def brake_mode(self, throttle, pitch, yaw, roll):
        """In brake mode, the vehicle attempts to brake all horizontal
        velocity as quickly as possible, engaging Position Hold mode
        after it stops.

        """
        if abs(self.velocity[0]) + abs(self.velocity[2]) < self.config['brake_velocity']:
            self.set_poshold_mode()
            return

        return self.brake()

    def brake(self):
        self.velocity_sp[0] = 0
        self.velocity_sp[1] = 0
        self.velocity_sp[2] = 0

        return self.velocity_mode(0, 0, 0, 0)

    def set_hybrid_mode(self):
        self.mode = self.hybrid_mode

    def hybrid_mode(self, throttle, pitch, yaw, roll):
        """Hybrid Mode is a blend of rate and stabilize mode. When the pitch
        and roll axies are below the threshold value, the vehicle is
        controlled as if in stabilize mode. When they go above the
        threshold value, the vehicle is controlled as if in rate
        mode. With this you can do rolls and flips while still
        self-levelling when the controls are released.

        """
        if abs(pitch) > self.config['hybrid_pitch'] or abs(roll) > self.config['hybrid_roll']:
            return self.rate_mode(throttle, pitch, yaw, roll)

        else:
            return self.stabilize_mode(throttle, pitch, yaw, roll)

    def get_collisions(self):
        # get horizontal velocity relative to our y rotation
        d = self.quat_ry * self.velocity

        self.collision_points = [self.position + (d.normalized * v) for v in self.collision_radius]

        if any(self.colmarks):
            self.colmarks[0].Move(self.collision_points[0])
            self.colmarks[1].Move(self.collision_points[1])
            self.colmarks[2].Move(self.collision_points[2])
        else:
            self.colmarks[0] = Besiege.CreateMark(self.collision_points[0])
            self.colmarks[1] = Besiege.CreateMark(self.collision_points[1])
            self.colmarks[2] = Besiege.CreateMark(self.collision_points[2])

            self.colmarks[0].SetColor(Color(0, 1, 0))
            self.colmarks[1].SetColor(Color(0, 1, 0))
            self.colmarks[2].SetColor(Color(0, 1, 0))

        near = Physics.Linecast(self.collision_points[0], self.collision_points[1])
        mid = Physics.Linecast(self.collision_points[1], self.collision_points[2])

        if near:
            self.colmarks[0].SetColor(Color(1, 0, 0))
            self.colmarks[1].SetColor(Color(1, 0, 0))
        else:
            self.colmarks[0].SetColor(Color(0, 1, 0))
            self.colmarks[1].SetColor(Color(0, 1, 0))

        if mid:
            self.colmarks[1].SetColor(Color(1, 0, 0))
            self.colmarks[2].SetColor(Color(1, 0, 0))

        else:
            self.colmarks[1].SetColor(Color(0, 1, 0))
            self.colmarks[2].SetColor(Color(0, 1, 0))

        Besiege.Watch('colzones', str((near, mid)))

        return near, mid

    @property
    def terrain(self):
        # raycast straight down and get the real altitude
        p = Vector3(self.position[0],
                    self.position[1] - self.core_height,
                    self.position[2])

        try:
            hit = Besiege.GetRaycastHit(p, Vector3.down)
        except Exception:
            return 0

        return hit.y

    @property
    def velocity(self):
        # velocity relative to the vehicle's yaw, disregarding other
        # axes
        return self.world_to_local_y_rot(self.core.Velocity)

    @property
    def position_sp_distance(self):
        # vertical distance must be signed
        v = (Vector3(0, self.position_sp.y, 0) - Vector3(0, self.position.y, 0)).y

        h = Vector3.Distance(
            Vector3(self.position.x, 0, self.position.z),
            Vector3(self.position_sp.x, 0, self.position_sp.z))

        d = Vector3.Distance(self.position_sp, self.position)

        return Vector3(h, v, d)

    @property
    def position_sp_local(self):
        return self.world_to_local_y_rot(self.position_sp - self.position)

    @property
    def quat_ry(self):
        return Quaternion.Euler(Vector3(0, self.rotation.y, 0))

    def world_to_local_y_rot(self, v):
        # rotate vector with the vehicle's current yaw
        quat = Quaternion.Inverse(self.quat_ry)

        return quat * v

    def set_mark(self):
        try:
            p = Besiege.GetRaycastHit()
        except Exception:
            return

        p[1] += 30

        self.waypoints.append(Waypoint(p))

    def clear_mark(self):
        if self.waypoints:
            self.waypoints.pop().clear()



class Plane(BasePlane):
    def set_power(self, throttle_adj, pitch_adj, yaw_adj, roll_adj):

        power = {
            'motor_pitch': pitch_adj,
            'motor_yaw': yaw_adj,
            'motor_roll': roll_adj,
            }

        for m in self.motors['motor_throttle']:
            m.SetSliderValue('FLYING SPEED', abs(throttle_adj))

            if throttle_adj > 0:
                m.SetToggleMode('REVERSE', False)
            elif throttle_adj < 0:
                m.SetToggleMode('REVERSE', True)

        for k, v in power.items():
            for m in self.motors[k]:
                m.SetAngle(power[k])
