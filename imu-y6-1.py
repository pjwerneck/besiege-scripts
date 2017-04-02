

from bsg.rotor import Y6


Update = Y6(
    hovering_speed=1,

    motors={
        'l1': ['SPINNING 1'],
        'r1': ['SPINNING 2'],
        'b1': ['SPINNING 3'],

        'l2': ['SPINNING 5'],
        'r2': ['SPINNING 6'],
        'b2': ['SPINNING 4'],
        },

    rate_gain_pitch=(0.0266, 0, 0),
    rate_gain_yaw=(0.1, 0, 0),
    rate_gain_roll=(0.0266, 0, 0),

    yaw_gain=(0.066, 0.01, 0.0033),
    )
