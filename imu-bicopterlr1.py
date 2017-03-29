

from bsg.rotor import BicopterLR


FixedUpdate = BicopterLR(

    hovering_speed=1.13,

    motors={
        'l': ['SPINNING 1'],
        'r': ['SPINNING 2'],
        'pitch': ['STEERING 1', 'STEERING 2'],
        },

    rate_gain_pitch=(0.1, 0.1, 0),
    rate_gain_yaw=(0.01, 0, 0),
    rate_gain_roll=(0.01, 0, 0),

    yaw_gain=(0.066, 0.01, 0.0033),

    )
