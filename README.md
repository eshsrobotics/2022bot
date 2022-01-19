# 2022bot
Source code for the ESHS Robotics 2022 FRC competition robot.

This file contains motor and port assignments, the robot's control scheme, and technical discussion about its various subsystems.

## Control scheme ##

_Note_: for field-oriented swerve to work, the robot must start the match with
its front side facing away from the driver.__

- XBox controller:
  1. Left joystick: **Field-oriented strafing**
      Regardless of the orientation of the robot, pushing the left joystick
      away from you will cause it to drive away from you; moving the joystick
      to the left will cause the robot to move leftward, and so on.
  2. Right joystick:
     1. Horizontal channel: **Rotation**
         Moving the right joystick to the right turns the robot clockwise;
         moving it to the left turns the robot counterclockwise.
     2. Vertical channel: _Unused_

## Motor and port assignments ##
## Subsystems ##
### Input ###
### Drive ###
[As the team
did](https://github.com/eshsrobotics/2020bot/blob/master/src/main/java/frc/robot/subsystems/NewWheelDriveSubsystem.java)
for the 2019-2020 and 2020-2021 seasons, this year's robot will employ a
swerve drive using MK3 modules from Swerve Drive Specialties.  Eight brushless
NEO motors control the swerve modules.  They are daisy-chained in a CAN bus
using the following CAN IDs:



### Shooter ###

The shooter's purpose is to sink balls into the circular goal.  The goal is
ringed with reflective tape, and the center of that formation of reflective
quadrilaterals represents a **vision solution** that we can target with the
Limelight vision camera.

#### Shooter components ####

The shooter consists of a circular opening with a bottom flywheel adjacent to
it.  A hood behind the opening can be adjusted to fine-tune the trajectory of
the parabolic arc the launched cargo will travel in.

The goal in the competition field contains a rotating agitator in the center.
If you are thinking of the agitators that you would see in washing machines,
you are not far off the mark (except this one is larger and allows balls to
slip through.)  The agitator seems specifically designed to thwart attempts to
sink cargo in using high parabolic arcs with a lot of topspin, which is
typical for flywheel-based shooters.

In response, we added a second, rear flywheel to the design on 2022-01-19 to
reduce the topspin of the ball.  To simplify the resulting equations, the top
flywheel's speed is defined solely by the bottom flywheel's speed, making it a
**dependent variable**.

#### Shooting solution ####

### Intake ###
