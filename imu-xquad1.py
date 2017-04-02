

from bsg.rotor import XQuad


Update = XQuad(
    hovering_speed=1.16,

    motors={
        'fl': ['SPINNING 1'],
        'fr': ['SPINNING 2'],
        'bl': ['SPINNING 4'],
        'br': ['SPINNING 3'],
        },

    # Rate mode PID controller gain values, (Kp, Ki, Kd). Only Kp is
    # needed.
    rate_gain_pitch=(0.027, 0, 0),
    rate_gain_yaw=(0.027, 0, 0),
    rate_gain_roll=(0.027, 0, 0),

    # Stabilize mode PID controller gain values.
    stabilize_gain_pitch=(0.0266, 0, 0),
    stabilize_gain_yaw=(0.04, 0, 0),
    stabilize_gain_roll=(0.0266, 0, 0),

    # Velocity mode PID controller values. Velocity was tuned by
    # adjusting the Kp factor to get velocity to 3/4 of the setpoint,
    # then adding Kd until it oscillates and cutting it down to 1/3,
    # and finally by adjusting Ki until the setpoint is achieved
    # without overshooting.
    velocity_gain_x=(0.033, 0, 0.01),
    velocity_gain_y=(0.0266, 0.33, 0.0027),
    velocity_gain_z=(0.033, 0, 0.01),

    # Altitude and Position Hold mode PID gain values. These
    # gains go through the velocity, stabilize and rate PIDs, so they
    # should be very small values. Ki is kept at zero to avoid overshooting,
    # which is undesirable when flying autonomously.
    althold_gain=(0.033, 0, 0.021),
    poshold_gain_x=(0.0042, 0, 0.0005),
    poshold_gain_z=(0.0042, 0, 0.0005),

    # Auto mode PID values. These can be equal to poshold gain, or can
    # be adjusted for more aggressive piloting.
    auto_gain_x=(0.0066, 0, 0),
    auto_gain_y=(0.022, 0, 0),
    auto_gain_z=(0.0066, 0, 0),

    # Yaw control mode PID gain values. This is the controller used by
    # the vehicle to zero lateral velocity by adjusting yaw.
    yaw_gain=(0.0133, 0, 0),

    )
