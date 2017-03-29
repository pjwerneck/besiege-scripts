

from bsg.rotor import HexaX


FixedUpdate = HexaX(
    hovering_speed=1.07,

    motors={'1': ['SPINNING 1'],
            '2': ['SPINNING 2'],
            '3': ['SPINNING 3'],
            '4': ['SPINNING 4'],
            '5': ['SPINNING 5'],
            '6': ['SPINNING 6'],
            },

    rate_gain_pitch=(0.023, 0, 0),
    rate_gain_yaw=(0.039, 0, 0.0),
    rate_gain_roll=(0.023, 0, 0),

    yaw_gain=(0.066, 0.01, 0.0033),
    )
