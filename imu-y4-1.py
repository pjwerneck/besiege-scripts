

from bsg.rotor import Y4


Update = Y4(
    hovering_speed=1.06,

    motors={'l': ['SPINNING 1'],
            'r': ['SPINNING 2'],
            'bu': ['SPINNING 3'],
            'bd': ['SPINNING 4'],
            },

    rate_gain_pitch=(0.0266, 0, 0),
    rate_gain_yaw=(0.1, 0, 0),
    rate_gain_roll=(0.0266, 0, 0),

    yaw_gain=(0.033, 0, 0),
    )
