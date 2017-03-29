

from bsg.rotor import Dualcopter


FixedUpdate = Dualcopter(
    hovering_speed=1.15,

    motors={
        'u': ['SPINNING 1'],
        'd': ['SPINNING 2'],

        'pitch': ['STEERING 3', 'STEERING 4'],
        'roll': ['STEERING 1', 'STEERING 2'],
        },

    rate_gain_pitch=(0.1, 0, 0),
    rate_gain_yaw=(0.03, 0, 0),
    rate_gain_roll=(0.1, 0, 0),

    yaw_gain=(0.066, 0.01, 0.0033),

    )
