# besiege-scripts
Python scripts for Besiege creations


To use these scripts you need [Besiege](http://store.steampowered.com/app/346010/), and both the [LenchScripterMod](https://github.com/lench4991/LenchScripterMod) and [AdvancedControlsMod](https://github.com/lench4991/AdvancedControlsMod). 

This repository is a backup of my Besiege Scripts folder, but you can copy the `bsg` directory to your own and use it as a library.

Thanks to [lench4991](https://github.com/lench4991) for the excellent work on those mods.

## R.O.T.O.R.

The `bsg.rotor` module implements a fully-featured controller for multicopters, with acrobatic and level flight modes, autopiloting with waypoints, and collision avoidance.

I tried to implement it following real-world controllers as closely as possible -- except for noise filtering, which is the annoying part anyway -- and it works well, at least well enough considering the weird aerodynamics and propeller drag in Besiege.


### Flight Modes

The following flight modes are available:


#### Rate Mode

In Rate Mode, pitch, yaw and roll axes control the rate of rotation in the respective axes. The throttle axis controls motor power directly.

#### Stabilize Mode

In stabilize mode, the pitch and roll axes control the current angle in the respective axes, with the vehicle self-levelling when they are released. The yaw axis works as if in rate mode, and holds the current yaw when released. Throttle works as if in rate mode.

#### Hybrid Mode

Hybrid Mode is a blend of rate and stabilize mode. When the pitch and roll axies are below the threshold value, the vehicle is controlled as if in stabilize mode. When they go above the threshold value, the vehicle is controlled as if in rate mode. With this you can do rolls and flips while still self-levelling when the controls are released.

#### Velocity Mode

In Velocity mode, the roll, throttle and pitch axes control velocity in the x, y, z axes, respectively, relative to the vehicle's current heading. Yaw works as if in the stabilize mode.

Piloting in the velocity mode is not recommended. This mode is used internally by the other autonomous modes.

#### Altitude Hold Mode

In Altitude Hold mode, the vehicle acts as if in stabilize mode, but trying to maintain the current altitude. The throttle axis changes the target altitude.

#### Position Hold Mode

In Position Hold mode, the vehicle holds the current horizontal position and altitude. Throttle adjusts altitude as if in Altitude Hold mode. Pitch and roll adjust the horizontal position. Yaw works as if in stabilize mode.

#### RTH Mode

In RTH (Return to Home) mode, the vehicle autopilots back to the initial horizontal position and engages Land mode. Controls are ignored.

#### Land Mode

In Land mode, the vehicle will enter Position hold mode until it stabilizes, then descend and land on the current position.

#### Auto Mode

In Auto mode, the vehicle will move autonomously to every waypoint, removing them from the list as they are reached. Once the list is empty, the vehicle switches to Position hold mode.

The autopilot is smart enough to detect obstacles between waypoints and plot a new course around them if necessary. 

#### Brake Mode

In brake mode, the vehicle attempts to zero all horizontal velocity as quickly as possible, engaging Position Hold mode after it stops.


### Yaw Modes

Yaw is controlled by its own modes, independently of the current flight mode.

#### Yaw Null

Yaw is controlled as if in rate mode. The yaw axis changes current rate of rotation.

#### Yaw Lock

The yaw axis changes the current yaw as if in rate mode, and the current rotation is maintained when the stick is released.

#### Yaw Heading

Yaw is automatically adjusted so the vehicle's heading is the same as the velocity vector, cancelling all lateral velocity. With this you can fly a multicopter with only a single dual-axis stick, as if it were a plane.



### Models

I recommend using the starting block as the multicopter core and keeping it at the same level as the propellers.

If you're in doubt about which side is the front, click the `Translate Machine` button. The blue arrow points to the front.

Here's a list of configurations available. I did extensive testing and tuning only with the `rotor.XQuad` class, but all of them should be stable in rate and plane modes.

#### `rotor.XQuad`

For quadcopters in the X configuration.

- `motor_fl`: front left, clockwise
- `motor_fr`: front left, counter-clockwise
- `motor_bl`: rear left, counter-clockwise
- `motor_br`: rear right, clockwise

#### `rotor.PlusQuad`

For quadcopters in the + configuration.

- `motor_f`: front, clockwise
- `motor_b`: rear, clockwise
- `motor_l`: left, counter-clockwise
- `motor_r`: right, counter-clockwise

#### `rotor.Tricopter` 

For tricopters in the Y or T configuration, with yaw servo. WIP.

#### `rotor.Y6`

For hexacopters in the Y configuration.

- `motor_l1`: up left, clockwise
- `motor_r1`: up right, counter-clockwise
- `motor_b1`: up rear, counter-clockwise
- `motor_l1`: down left, counter-clockwise
- `motor_r1`: down right, clockwise
- `motor_b1`: down rear, clockwise

#### `rotor.Y4`

For hexacopters in the Y configuration.

- `motor_l`: left, clockwise
- `motor_r`: right, counter-clockwise
- `motor_bu`: up rear, counter
- `motor_bd`: down rear, counter-clockwise


#### `rotor.BicopterLR`

For bicopters with left/right motor and pitch servos. As of now, they are stable only in rate and plane mode.

- `motor_l`: left, clockwise
- `motor_r`: right, counter-clockwise
- `motor_pitch`: pitch steering, clockwise.


### Configuration

There are many configuration parameters. The script files has detailed descriptions for each one.

### Tuning

If you never tuned a multicopter, I have bad news for you: it's the most boring and annoying part.

If you fly multicopters and tuned a lot of them, I have good news for you: because there are no wind or pressure disturbances and no sensor noise, you can fly well after tuning only the Kp parameters. Tuning Kd might give you a more locked-in feeling, but in most flight modes Ki isn't necessary and will add overshoot, which is undesirable. 

Since the flight modes are just a series of cascaded PID controllers, you should tune them in the following order:


- rate
- stabilize
- velocity
- althold
- poshold
- auto
- plane




