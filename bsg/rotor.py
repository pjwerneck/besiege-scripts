

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
from .utils import pretty


class Waypoint(object):
    __slots__ = ['position', 'mark']

    def __init__(self, position, mark=None):
        self.position = position
        self.mark = mark or Besiege.CreateMark(position)

        if self.mark:
            self.mark.SetColor(Color.green)

    def clear(self):
        if self.mark:
            self.mark.Clear()


class BaseRotor(object):

    yaw_mode = 'lock'

    def __init__(self, **config):
        clear_marks()

        self.hover = config['hovering_speed']
        self.core = config['core_block']
        self.core_height = None

        self.motors = {k: v for (k, v) in config.iteritems() if k.startswith('motor_')}

        self.axes = {'throttle': config['axis_throttle'],
                     'pitch': config['axis_pitch'],
                     'yaw': config['axis_yaw'],
                     'roll': config['axis_roll'],
                     }

        self.keys = {k.split('_', 1)[1]: v for (k, v) in config.iteritems() if k.startswith('key_')}

        self.mode = [self.rate_mode]

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
            pid.AngularPID(config['stabilize_gain_pitch'], limit_i=mm(1)),
            pid.AngularPID(config['stabilize_gain_yaw'], limit_i=mm(1)),
            pid.AngularPID(config['stabilize_gain_roll'], limit_i=mm(1)),
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
            pid.PID(config['poshold_gain_z'], limit=mm(1)),
            )

        self.auto_pid = (
            pid.PID(config['auto_gain_x'], limit=mm(1)),
            pid.PID(config['auto_gain_y'], limit=mm(1)),
            pid.PID(config['auto_gain_z'], limit=mm(1)),
            )

        self.plane_mode_pid = pid.PID(config['plane_gain'], limit=mm(1))

        self.home = None

        self._hold_yaw = None

        clear_marks()

        self.waypoints = []

        self.waypoints = [
            Waypoint(Vector3(-252.1, 76, 78.4)),
            Waypoint(Vector3(-353.5, 123.4, 405.6)),
            Waypoint(Vector3(17, 181, -7)),
            Waypoint(Vector3(311, 181, 294)),
            Waypoint(Vector3(176.9, 243.2, 314.7)),
            Waypoint(Vector3(0, 2, 0)),
            ]

        self.current_waypoint = None
        self.config = config

        self._first_update = True

        self._colmarks = [None, None]
        self._avdmarks = [None, None]

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

        #self.get_collisions()

        # A mode function can return None if it's delegating to
        # another mode that was appended to the stack, or if it's
        # leaving the top of the stack. In that case, we loop until we
        # get a power adjustment response
        power = None
        while power is None:
            power = self.mode[-1](*self.controls)

        Besiege.Watch('dt', self.dt)
        Besiege.Watch('mode', self.mode[-1].__name__)

        Besiege.Watch('controls', pretty(self.controls))

        Besiege.Watch('rate', self.rate)
        Besiege.Watch('rate_sp', self.rate_sp)

        Besiege.Watch('rotation', self.rotation)
        Besiege.Watch('rotation_sp', self.rotation_sp)

        Besiege.Watch('position', self.position)
        Besiege.Watch('position_sp', self.position_sp)
        Besiege.Watch('position_sp_local', self.position_sp_local)
        Besiege.Watch('position_sp_distance', self.position_sp_distance)

        Besiege.Watch('velocity', self.velocity)
        Besiege.Watch('velocity_sp', self.velocity_sp)

        # get ASL and real altitude
        Besiege.Watch('altitude', Vector2(self.position.y, self.position.y - self.terrain))

        self.set_power(*power)

    def set_rate_mode(self):
        self.mode[0] = self.rate_mode

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
        self.mode[0] = self.stabilize_mode

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
        self.mode[0] = self.velocity_mode

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
        self.mode[0] = self.althold_mode

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
        self.mode[0] = self.poshold_mode

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
        self.mode[0] = self.rth_mode

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
        self.mode[0] = self.land_mode

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
        self.mode[0] = self.auto_mode

    def auto_mode(self, throttle, pitch, yaw, roll):
        """In Auto mode, the vehicle will move autonomously to every waypoint,
        removing them from the list as they are reached. Once the list
        is empty, the vehicle switches to Position hold mode.

        """

        if self.current_waypoint:
            if self.position_sp_distance[2] > self.config['auto_radius']:
                return self.goto_position()

            else:
                self.current_waypoint.clear()
                self.current_waypoint = None

        if self.current_waypoint is None:
            if self.waypoints:
                if self.set_next_waypoint():
                    self.position_sp = self.current_waypoint.position
                    return self.poshold_mode(0, 0, 0, 0)

            self.set_poshold_mode()

    def set_next_waypoint(self):
        # get the waypoint direction
        b = self.waypoints[0].position
        d = (b - self.position).normalized

        a = self.position + (d * 10)

        # check for collisions between the current position and the waypoint
        if not Physics.Linecast(a, b):
            self.current_waypoint = self.waypoints.pop(0)
            return True

        # if there's a collision, try to find a clear path above the obstacle
        a2 = Vector3(a[0], a[1], a[2])
        b2 = Vector3(b[0], b[1], b[2])

        for y in xrange(0, 1000, 10):
            a2 = Vector3(a[0], a[1] + y, a[2])
            b2 = Vector3(b[0], a[1] + y, b[2])

            segs = [Physics.Linecast(a, a2), Physics.Linecast(a2, b2), Physics.Linecast(b2, b)]

            if any(segs):
                continue

            self.waypoints.insert(0, Waypoint(b2))
            self.current_waypoint = Waypoint(a2)
            return True

        Besiege.Watch("panic", "set next waypoint")

    def goto_position(self):
        h, v, t = self.position_sp_distance

        x, y, z = self.position_sp_local

        self.auto_pid[0].gain_f = 0.8
        self.auto_pid[1].gain_f = 0.8
        self.auto_pid[2].gain_f = 0.8

        # TODO: have to find a dynamic p value that gives a smooth
        # ascent and descent curve.
        p = 0.2

        # if below target, prioritize climbing, so lower xz gain
        if h > 10 and v > 10:
            self.auto_pid[0].gain_f = p
            self.auto_pid[2].gain_f = p

        # if above target, prioritize moving, so lower y gain
        if h > 10 and v < -10:
            self.auto_pid[1].gain_f = p

        roll_adj = self.auto_pid[0].update(self.position_sp_local[0], self.dt)
        throttle_adj = self.auto_pid[1].update(-self.position_sp_local[1], self.dt)
        pitch_adj = self.auto_pid[2].update(-self.position_sp_local[2], self.dt)

        if self.yaw_mode == 'lock':
            yaw_adj = 0

        elif self.yaw_mode == 'heading':
            yaw_adj = -self.plane_mode_pid.update(self.velocity.x, self.dt)

        return self.velocity_mode(throttle_adj, pitch_adj, yaw_adj, roll_adj)

    def set_plane_mode(self):
        self.mode[0] = self.plane_mode
        self.plane_mode_pid.reset()

    def plane_mode(self, throttle, pitch, yaw, roll):
        """The Plane mode works like rate mode, but the vehicle can be
        controlled using only the pitch and roll axes. It will
        automatically adjust yaw to zero all lateral velocity.

        """

        v = self.world_to_local(self.core.Velocity)

        if self.velocity.z > self.config['plane_min_z']:
            yaw = -self.plane_mode_pid.update(v.x, self.dt)

        return self.rate_mode(throttle, pitch, yaw, roll)

    def set_brake_mode(self):
        self.mode[0] = self.brake_mode

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

    def set_panic_brake_mode(self):
        self.mode[0] = self.panic_brake_mode

        self.position_sp[1] = self.position[1]
        self.althold_pid.reset()

        self._hold_yaw = None

    def panic_brake_mode(self, throttle, pitch, yaw, roll):
        """Panic Brake mode is engaged when the autopilot cannot resolve a path
        around an obstacle or when a collision can't be avoided in
        time. The vehicle brakes even harder than in brake mode, and
        engages panic hold mode afterwards.

        """
        if abs(self.velocity[0]) + abs(self.velocity[2]) >= self.config['brake_velocity']:
            self.velocity_pid[0].gain_f = 2
            self.velocity_pid[1].gain_f = 2
            self.velocity_pid[2].gain_f = 2
            return self.brake()

        self.velocity_pid[0].gain_f = 1
        self.velocity_pid[1].gain_f = 1
        self.velocity_pid[2].gain_f = 1

        self.set_panic_hold_mode()

    def set_panic_hold_mode(self):
        self.mode[0] = self.panic_hold_mode
        self.position_sp.Set(self.position[0], self.position[1], self.position[2])

        self.poshold_pid[0].reset()
        self.poshold_pid[2].reset()

        self.althold_pid.reset()

        self._hold_yaw = None

    def panic_hold_mode(self, throttle, pitch, yaw, roll):
        return self.poshold_mode(0, 0, 0, 0)

    def set_hybrid_mode(self):
        self.mode[0] = self.hybrid_mode

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
        radius = self.config['collision_vehicle_radius']
        range_ = self.config['collision_range']

        vel = self.velocity
        # get velocity relative to our y rotation
        d = self.quat_ry * vel
        dn = d.normalized

        # the nearest point is fixed relative to the vehicle
        a = self.position + (dn * radius)
        b = self.position + (d * range_)

        if any(self._colmarks):
            self._colmarks[0].Move(a)
            self._colmarks[1].Move(b)
        else:
            self._colmarks[0] = Besiege.CreateMark(a)
            self._colmarks[1] = Besiege.CreateMark(b)

        hit = Physics.Linecast(a, b)

        c = Color.red if hit else Color.yellow

        self._colmarks[0].SetColor(c)
        self._colmarks[1].SetColor(c)

        # if there's a collision, find the y velocity change that clears us out of it
        if hit:
            for dv in xrange(10):
                tvel = Vector3(vel[0], vel[1] + dv, vel[2])
                dg = self.quat_ry * tvel
                dng = dg.normalized

                ag = self.position + (dng * radius)
                bg = self.position + (dg * range_)

                # if there's a hit, continue
                if Physics.Linecast(ag, bg):
                    continue

                if any(self._avdmarks):
                    self._avdmarks[0].Move(ag)
                    self._avdmarks[1].Move(bg)
                else:
                    self._avdmarks[0] = Besiege.CreateMark(ag)
                    self._avdmarks[1] = Besiege.CreateMark(b)

                    self._avdmarks[0].SetColor(Color.green)
                    self._avdmarks[1].SetColor(Color.green)

        else:
            if any(self._avdmarks):
                self._avdmarks[0].Clear()
                self._avdmarks[1].Clear()

                self._avdmarks = [None, None]

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
        # velocity relative to the vehicle's yaw, disregarding
        # rotation angles from other axes
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

    @property
    def quat(self):
        return Quaternion.Euler(self.rotation)

    def world_to_local_y_rot(self, v):
        # rotate vector with the vehicle's current yaw
        quat = Quaternion.Inverse(self.quat_ry)
        return quat * v

    def world_to_local(self, v):
        quat = Quaternion.Inverse(self.quat)
        return quat * v

    def add_waypoint(self):
        try:
            p = Besiege.GetRaycastHit()
        except Exception:
            return

        p[1] += 30

        self.waypoints.append(Waypoint(p))

    def delete_last_waypoint(self):
        if self.waypoints:
            self.waypoints.pop().clear()


class XQuad(BaseRotor):

    def set_power(self, throttle_adj, pitch_adj, yaw_adj, roll_adj):
        hover = self.hover

        power = {
            'motor_fl': hover + hover * (throttle_adj - pitch_adj - roll_adj - yaw_adj),
            'motor_fr': hover + hover * (throttle_adj - pitch_adj + roll_adj + yaw_adj),
            'motor_bl': hover + hover * (throttle_adj + pitch_adj - roll_adj + yaw_adj),
            'motor_br': hover + hover * (throttle_adj + pitch_adj + roll_adj - yaw_adj),
            }

        Besiege.Watch('power', pretty(power.values()))
        Besiege.Watch('avg_power', sum(power.values()) / 4)

        for k, v in power.items():
            v = pid.clip(v, -12, 12)
            for m in self.motors[k]:
                m.SetSliderValue('SPEED', v)


class PlusQuad(BaseRotor):

    def set_power(self, throttle_adj, pitch_adj, yaw_adj, roll_adj):
        hover = self.hover

        power = {
            'motor_f': hover + hover * (throttle_adj - pitch_adj - yaw_adj),
            'motor_b': hover + hover * (throttle_adj + pitch_adj - yaw_adj),
            'motor_l': hover + hover * (throttle_adj - roll_adj + yaw_adj),
            'motor_r': hover + hover * (throttle_adj + roll_adj + yaw_adj),
            }

        Besiege.Watch('power', Vector4(*power.values()))
        Besiege.Watch('avg_power', sum(power.values()) / 4)

        for k, v in power.items():
            v = pid.clip(v, -12, 12)
            for m in self.motors[k]:
                m.SetSliderValue('SPEED', v)


class Y6(BaseRotor):

    yaw_mode = 'heading'

    def set_power(self, throttle_adj, pitch_adj, yaw_adj, roll_adj):
        hover = self.hover

        power = {
            'motor_l1': hover + hover * (throttle_adj - pitch_adj - roll_adj - yaw_adj),
            'motor_l2': hover + hover * (throttle_adj - pitch_adj - roll_adj + yaw_adj),

            'motor_r1': hover + hover * (throttle_adj - pitch_adj + roll_adj + yaw_adj),
            'motor_r2': hover + hover * (throttle_adj - pitch_adj + roll_adj - yaw_adj),

            'motor_b1': hover + hover * (throttle_adj + pitch_adj + yaw_adj),
            'motor_b2': hover + hover * (throttle_adj + pitch_adj - yaw_adj),
            }

        Besiege.Watch('power', '(' + ', '.join(['%0.2f' % x for x in power.values()]) + ')')
        Besiege.Watch('avg_power', sum(power.values()) / len(power))

        for k, v in power.items():
            v = pid.clip(v, -12, 12)
            for m in self.motors[k]:
                m.SetSliderValue('SPEED', v)


class Tricopter(BaseRotor):

    def set_power(self, throttle_adj, pitch_adj, yaw_adj, roll_adj):
        hover = self.hover

        power = {
            'motor_l': hover + hover * (throttle_adj - pitch_adj - roll_adj),
            'motor_r': hover + hover * (throttle_adj - pitch_adj + roll_adj),
            'motor_b': hover + hover * (pitch_adj),
            }

        Besiege.Watch('power', pretty(power.values()))

        for k, v in power.items():
            v = pid.clip(v, -12, 12)
            for m in self.motors[k]:
                m.SetSliderValue('SPEED', v)

        yaw_adj = pid.clip(yaw_adj, -15, 15)

        Besiege.Watch('yaw_adj', yaw_adj)

        self.motors['motor_yaw_servo'][0].SetAngle(yaw_adj)


class BicopterLR(BaseRotor):

    def set_power(self, throttle_adj, pitch_adj, yaw_adj, roll_adj):
        hover = self.hover

        power = {
            'motor_l': hover + hover * (throttle_adj - roll_adj),
            'motor_r': hover + hover * (throttle_adj + roll_adj),
            }

        Besiege.Watch('power', pretty(power.values()))

        for k, v in power.items():
            v = pid.clip(v, -12, 12)
            for m in self.motors[k]:
                m.SetSliderValue('SPEED', v)

        self.motors['motor_pitch'][0].SetAngle(pitch_adj + yaw_adj)
        self.motors['motor_pitch'][1].SetAngle(pitch_adj - yaw_adj)
