# 2022bot
Source code for the ESHS Robotics 2022 FRC Rapid React competition robot.

The robot's tentative name is **TATR** (The Articulated Turret Robot.)

This file contains [motor and port assignments](#motor-and-port-assignments), the robot's [control scheme](#control-scheme), and technical discussion about its various [subsystems](#subsystems).

## Avaliable Motors ##

Motor distribution is as follows:

Drive 
  - 8 NEO-Spark MAX

Shooter
  - 2 NEO-Spark MAX
  - 2 Linear servo - Rev servo Hub

Turret
  - 1 NEO-550-Spark MAX

Intake
  - 1 mini-cim-Spark

Uptake
  - 1 mini-cim-Spark

Indexer
  - 2 NEO-550-Spark MAX

Climber
  - 1 NEO-Spark MAX
  - 1 cim spark
  
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

### CAN bus IDs ###

1. Front left drive motor (Brushless NEO @ Spark MAX)
2. Back left drive motor (Brushless NEO @ Spark MAX)
3. Back right drive motor (Brushless NEO @ Spark MAX)
4. Front right drive motor (Brushless NEO @ Spark MAX)
5. Front left pivot motor (Brushless NEO @ Spark MAX)
6. Back left pivot motor (Brushless NEO @ Spark MAX)
7. Back right pivot motor (Brushless NEO @ Spark MAX)
8. Front right pivot motor (Brushless NEO @ Spark MAX)

### PWM ports ###

## Subsystems ##
### Drive ###
[As the team
did](https://github.com/eshsrobotics/2020bot/blob/master/src/main/java/frc/robot/subsystems/NewWheelDriveSubsystem.java)
for the 2019-2020 and 2020-2021 seasons, this year's robot will employ a
swerve drive using MK3 modules from Swerve Drive Specialties.  Eight brushless
NEO motors control the swerve modules; their Spark MAX motor controllers are
daisy-chained in a CAN bus, allowingh us to use the CANSparkMAX [CANEncoder](https://codedocs.revrobotics.com/java/com/revrobotics/canencoder)
class to access the default encoders for the pivot wheels' motor controllers.

### Shooter ###
![Shooter subsystem, without hood motors or upper flywheel](docs/2022-01-17-frc-shooter.png)
The shooter's purpose is to sink balls into the circular goal.  The goal is
ringed with reflective tape, and the center of that formation of reflective
quadrilaterals represents a **vision solution** that we can target with the
Limelight vision camera.

#### Shooter components ####

1. **Limelight.**  The Limelight is a versatile and easy-to-configure camera
   with built-in LED headlights designed to quickly identify reflective tape
   targets.  Once identified, it asynchronously calculates five parameters and
   returns them to the rest of the robot code via `NetworkTables`:
    - `tv`: Whether the Limelight has any valid targets; 0 or 1.
    - `tx`: Horizontal offset from crosshair to target; ranges from -27.0° to
      27.0°.
    - `ty`: Vertical offset from crosshair to target; ranges from -20.5° to
      20.5°.
    - `ta`: Target area in terms of percentage of the image.  Ranges from 0.0
      to 1.0.
    - `ts`: Target skew or rotation.  Ranges from -90.0° to 0.0°.

    From these values, we can use trigonometry to obtain the **solution
    distance**, and then feed that parameter into the [shooting
    algorithm](#shooting-algorithm).
2. **Flywheels and hood.** The shooter consists of a circular opening with a
   bottom flywheel adjacent to it.  A hood behind the opening can be adjusted
   to fine-tune the trajectory of the parabolic arc the launched cargo will
   travel in.

   The goal in the competition field contains a rotating agitator in the
   center.  If you are thinking of the agitators that you would see in washing
   machines, you are not far off the mark (except this one is larger and
   allows balls to slip through.)  The agitator seems specifically designed to
   thwart attempts to sink cargo in using high parabolic arcs with a lot of
   topspin, which is typical for flywheel-based shooters.

   In response, we added a second, rear flywheel to the design on 2022-01-19
   to reduce the topspin of the ball.

#### Shooting algorithm ####

- **Independent variables (inputs).**  There is one under consideration and a
   few others that we are ignoring:

    1. **Distance (d):** The diagonal distance from the Limelight camera to the
       center of the vision target.  The units don't actually matter.
    1. ~~Height: The distance from the Limelight camera to the ground plane.~~

        Always assumed to be constant.
    1. ~~Azimuth: The horizontal deviation in degrees from the center of the target.~~

        Because the shooter subsystem is always supposed to aim at the center of
        the vision solution, we can assume that this is a constant with a value
        of 0.0.

- **Dependent variables (outputs).**  These variables must be expressed as
  functions of the independent variables, and are directly related to the ball
  trajectory.  We have two variables under consideration and one that we are
  ignoring.

    1. **Bottom flywheel speed (s)**: The voltage to apply to the bottom flywheel.

        This controls the strength of the shot, and is achieved by
        compressing the cargo within the shooter.

    1. **Hood angle (θ)**: The amount by which the hood curls over the ball.

        This controls the steepness of the shot's trajectory.

    1. ~~Top flywheel speed: The voltage to apply to the top flywheel~~.

        Reduces topspin.  We plan to always make this a multiple of the bottom
        flywheel's speed, so it does not need to be considered in our model.

So our goal is to determine the two equations *F* and *G* so that
```
    s = F(d)
    θ = G(d)
```
will correctly launch the ball into the target from the given shooting distance `d`.

#### Gathering data ####

To gather enough data to solve the unknown equations for the dependent
variables, we need to position the robot (really, position the *Limelight*) at
fixed distances and heights and then attempt to find a flywheel speed and hood
angle that will reliably sink the shot.  The indexer should only release the
ball to the shooter once the flywheel has accelerated to the correct speed.

At the time of writing, we do not know what the relationship between the
distance and the independent variables is, other than that we predict that the
functions will monotonically increase as the distance increases.

### Intake ###
![Drive and intake subsystems, including roller positions but without intake motors](docs/2022-01-17-frc-chassis.png)

The intake subsystem is roughly divided into two parts:

1. The intake rollers, which use 3D-printed miniature Mecanum wheels to
direct any cargo placed beneath them to the maw of the chassis; and
2. The *indexer*, which is the portion of the intake that can suspend a ball
   until the human driver decides to release it to the [shooter](#shooter).

For the intake rollers to work properly, cargo must be trapped in front of the
robot and pressed beneath the rollers so that they roll along the front
bumpers toward the front center.  The intake rollers can be reversed to
*reject* a ball that is of the wrong color; to automate this process, a color
sensor is positioned beneath the top of the intake to ascertain the color of
the ball.
