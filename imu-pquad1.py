

from bsg.rotor import PlusQuad


FixedUpdate = PlusQuad(
    hovering_speed=1.13,

    motors={
        'f': ['SPINNING 1'],
        'b': ['SPINNING 2'],
        'r': ['SPINNING 3'],
        'l': ['SPINNING 4'],
        },

    rate_gain_pitch=(0.023, 0, 0),
    rate_gain_yaw=(0.039, 0, 0.0),
    rate_gain_roll=(0.023, 0, 0),

    # Stabilize mode PID controller gain values.
    stabilize_gain_pitch=(0.0115, 0, 0),
    stabilize_gain_yaw=(0.05, 0, 0.002),
    stabilize_gain_roll=(0.0115, 0, 0),

    # Velocity mode PID controller values. Velocity was tuned by
    # adjusting the Kp factor to get velocity to 3/4 of the setpoint,
    # then adding Kd until it oscillates and cutting it down to 1/3,
    # and finally by adjusting Ki until the setpoint is achieved
    # without overshooting.
    velocity_gain_x=(0.024, 0, 0),
    velocity_gain_y=(0.069, 0, 0),
    velocity_gain_z=(0.024, 0, 0),

    # Altitude and Position Hold mode PID gain values. These
    # gains go through the velocity, stabilize and rate PIDs, so they
    # should be very small values. Ki is kept at zero to avoid overshooting,
    # which is undesirable when flying autonomously.
    althold_gain=(0.018, 0, 0.01),
    poshold_gain_x=(0.0048, 0, 0.00044),
    poshold_gain_z=(0.0048, 0, 0.00044),

    # Auto mode PID values.
    auto_gain_x=(0.0072, 0, 0.0005),
    auto_gain_y=(0.022, 0, 0.01),
    auto_gain_z=(0.0072, 0, 0.0005),

    # Plane mode PID gain values. This is the controller used by the
    # vehicle to zero all lateral velocity by adjusting yaw.
    yaw_gain=(0.066, 0.01, 0.0033),

    )
