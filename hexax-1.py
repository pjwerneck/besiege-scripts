



from bsg.rotor import HexaX


CONFIG = {
    # The motor speed at which the vehicle hovers, maintaining current
    # altitude.
    'hovering_speed': 1.07,

    # The core block. This isn't necessarily the starting block, but
    # it should be at or very close to the center of mass and at the
    # same level as the propellers.
    'core_block': Besiege.GetBlock('STARTING BLOCK 1'),

    # The core block height relative to the landing legs. If this is
    # not set, it's assumed to be the initial height
    'core_height': 2,

    # The four motors. Rotation direction should be set at the
    # vehicle. The code assumes fl,br=CW, fr,bl=CCW.
    'motor_1': [Besiege.GetBlock('SPINNING 1')],
    'motor_2': [Besiege.GetBlock('SPINNING 2')],
    'motor_3': [Besiege.GetBlock('SPINNING 3')],
    'motor_4': [Besiege.GetBlock('SPINNING 4')],
    'motor_5': [Besiege.GetBlock('SPINNING 5')],
    'motor_6': [Besiege.GetBlock('SPINNING 6')],


    # The Advanced Controls Mod axes used to control the vehicle. All
    # axes should go from -1 to 1. Make sure to include a reasonable
    # deadzone if you're using analog sticks.
    'axis_throttle': AdvancedControls.GetAxis('pid_altitude'),
    'axis_pitch': AdvancedControls.GetAxis('pid_pitch'),
    'axis_roll': AdvancedControls.GetAxis('pid_roll'),
    'axis_yaw': AdvancedControls.GetAxis('pid_yaw'),

    # Keys used to switch between modes and other commands
    'key_set_rth_mode': [KeyCode.Home],
    'key_set_land_mode': [KeyCode.End],

    'key_set_brake_mode': [KeyCode.Keypad0],
    'key_set_rate_mode': [KeyCode.Keypad1],
    'key_set_stabilize_mode': [KeyCode.Keypad2],
    'key_set_althold_mode': [KeyCode.Keypad3],
    'key_set_poshold_mode': [KeyCode.Keypad4],
    'key_set_hybrid_mode': [KeyCode.Keypad5],
    'key_set_velocity_mode': [KeyCode.Keypad6],
    'key_set_plane_mode': [KeyCode.Keypad7],
    'key_set_auto_mode': [KeyCode.Keypad8],

    'key_set_panic_brake_mode': [KeyCode.Keypad9],

    'key_add_waypoint': [KeyCode.Insert],
    'key_delete_last_waypoint': [KeyCode.Delete],

    # Rate mode PID controller gain values, (Kp, Ki, Kd).
    'rate_gain_pitch': (0.023, 0, 0),
    'rate_gain_yaw': (0.039, 0, 0.0),
    'rate_gain_roll': (0.023, 0, 0),

    # Multiplier applied to the axes values in rate mode. This is the
    # desired rate in degrees/s when the stick is at the max
    # position.
    'rate_cf_pitch': 180,
    'rate_cf_yaw': 180,
    'rate_cf_roll': 180,

    # Multiplier applied to the throttle axis in rate mode.
    'rate_cf_throttle': 5,

    # Stabilize mode PID controller gain values.
    'stabilize_gain_pitch': (0.0115, 0, 0),
    'stabilize_gain_yaw': (0.05, 0, 0.002),
    'stabilize_gain_roll': (0.0115, 0, 0),

    # Multiplier applied to the axes values in stabilize mode. This is
    # the vehicle angle in the given axis when the stick is at the max
    # position. There's no setting for yaw because yaw changes in
    # stabilize mode are handled by rate mode.
    'stabilize_cf_pitch': 45,
    'stabilize_cf_roll': 45,

    # This is the max vehicle angle in stabilize mode.
    'stabilize_max_pitch': 45,
    'stabilize_max_roll': 45,

    # Velocity mode PID controller values. Velocity was tuned by
    # adjusting the Kp factor to get velocity to 3/4 of the setpoint,
    # then adding Kd until it oscillates and cutting it down to 1/3,
    # and finally by adjusting Ki until the setpoint is achieved
    # without overshooting.
    'velocity_gain_x': (0.024, 0, 0.002),
    'velocity_gain_y': (0.069, 0.033, 0.001),
    'velocity_gain_z': (0.024, 0, 0.002),

    # Multiplier added to the axes values in velocity mode.
    'velocity_cf_x': 40,
    'velocity_cf_y': 20,
    'velocity_cf_z': 40,

    # Altitude and Position Hold mode PID gain values. These
    # gains go through the velocity, stabilize and rate PIDs, so they
    # should be very small values. Ki is kept at zero to avoid overshooting,
    # which is undesirable when flying autonomously.
    'althold_gain': (0.018, 0, 0.01),
    'poshold_gain_x': (0.0048, 0, 0.00044),
    'poshold_gain_z': (0.0048, 0, 0.00044),

    # Multiplier applied to the axes in altitude and position hold mode. This is
    # the desired rate in m/s at which the target position will
    # change, relative to the vehicle's current orientation.
    'althold_cf': 50,
    'poshold_cf_x': 20,
    'poshold_cf_z': 20,

    # Auto mode PID values.
    'auto_gain_x': (0.0072, 0, 0.0005),
    'auto_gain_y': (0.022, 0, 0.01),
    'auto_gain_z': (0.0072, 0, 0.0005),

    # How close close the vehicle must get to the current waypoint
    # before going to the next waypoint or engaging poshold mode.
    'auto_radius': 10.0,

    # The mininum altitude the vehicle will climb to while moving
    # towards the home location in rth mode.
    'rth_min_y': 30,

    # Plane mode PID gain values. This is the controller used by the
    # vehicle to zero lateral velocity by adjusting yaw.
    'plane_gain': (0.066, 0.01, 0.0033),

    # The minimum forward velocity required by plane mode to adjust
    # yaw. This avoids having the quad spinning in place when stopped.
    'plane_min_z': 2,

    # The minimum absolute axis values where switching to rate mode occurs.
    'hybrid_pitch': 0.9,
    'hybrid_roll': 0.9,

    # The maximum acceptable relative horizontal velocity for a
    # vehicle in brake mode. This is the velocity at which poshold
    # mode is engaged.
    'brake_velocity': 1,

    # The minimum radius around the vehicle for each collision
    # range. The whole vehicle must fit within the first radius.
    'collision_ranges': (10, 60, 110, 210),
    }


quad = HexaX(**CONFIG)


def Update():
    try:
        quad.update()
    except Exception as exc:
        import sys
        typ, val, tb = sys.exc_info()

        Besiege.Watch('exc', str(typ))
        Besiege.Watch('msg', val.message[-20:])
        Besiege.Watch('file', tb.tb_frame.f_code.co_filename.split('/')[-1])
        Besiege.Watch('line', tb.tb_lineno)

        raise
