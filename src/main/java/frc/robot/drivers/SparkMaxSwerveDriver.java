package frc.robot.drivers;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;

public class SparkMaxSwerveDriver implements SwerveDriver {
    /**
     * Stores values so we can see the values of delta degrees
     */
    private List<NetworkTableEntry> entries;

     /**
     * Controls the speed of the four swerve wheels.
     */
    private List<CANSparkMax> speedMotors;

    /**
     * This is an array of 4 boolean values, one for each of the 4 speed motors.
     * Whenever the reversal flag is true for a motor, then we will reverse the
     * motor's direction in software.
     */
    // How come you put "List" insted of "ArrayList", are they different? - Sam.
    private List<Boolean> reversalFlags;

    /**
     * Controls the angle of the four swerve wheels.
     */
    private List<CANSparkMax> pivotMotors;

    /**
     * Rotates the pivot motors to a given set point using PID.
     */
    private List<ProfiledPIDController> pidControllers;

    /**
     * Converts inputs from PWM into dutyCycles between zero and one.
     *
     * <p>"Inputs from PWM?  What do you mean?"</p>
     *
     * <p>Well, we abandoned using {@link
     * com.revrobotics.SparkMaxRelativeEncoder REVLib's built-in encoders}
     * this year because we were tired of their quirk where they would
     * consider the pivot motor's current position at power-on to be the zero
     * position.  We wanted an absolute encoder rather than a relative one!
     * So we opted to purchase four break-out boards that would directly
     * convert encoder data from the Mark III swerve modules
     * <em>themselves</em> into PWM signals, and then we soldered wires to
     * send those signals into the RoboRIO's DIO ports.</p>
     *
     * <p>The {@link DutyCycle} objects help to interpret those PWM numbers as
     * a number between 0 and 1; these numbers are <strong>the absolute
     * angle</strong> of the swerve modules, and do not change when the robot
     * is rebooted.</p>
     */
    private List<DutyCycle> dutyCycles;

    /**
     * These angles and speeds is where our drive "wants" to end up.
     * But it still won't use them unless you call drive(getGoalStates()).
     */
    private SwerveModuleState[] goalStates;

    // Manually-determined PID for the pivot motors.
    //
    // It took a long time to come up with these values.  Are you a bad enough dude
    // to modify them?

    /**
     * If P is equal to 1/N, then when the measurement value is N degrees
     * away from the setpoint, we will apply 100% power to the pivot motor.
     *
     * For now, a value between 10 and 20 degrees will probably due.
     */
    private final double P = 2.0;

    /**
     * A good value for I is around 10% of P.
     */
    private final double I = 0;

    /**
     * D dampens the PID curve.  "D causes all kinds of problems."
     */
    private final double D = 0.0;

    /**
     * Initializes all of the {@link CANSparkMax} motor and PID controllers.
     */
    public SparkMaxSwerveDriver() {
        // Initialize the swerve motors (pivot and speed.)
        try {
            goalStates = reset(0);
            reversalFlags = new ArrayList<Boolean>();
            pivotMotors = new ArrayList<CANSparkMax>();
            speedMotors = new ArrayList<CANSparkMax>();
            dutyCycles = new ArrayList<DutyCycle>();
            entries = new ArrayList<NetworkTableEntry>();
            Collections.addAll(pivotMotors, new CANSparkMax[4]);
            Collections.addAll(speedMotors, new CANSparkMax [4]);
            Collections.addAll(reversalFlags, new Boolean[] { false, false, false, false });
            reversalFlags.set(Constants.FRONT_RIGHT, true);
            reversalFlags.set(Constants.BACK_RIGHT, true);
            Collections.addAll(dutyCycles, new DutyCycle[4]);
            Collections.addAll(entries, new NetworkTableEntry[16]);

            speedMotors.set(Constants.FRONT_LEFT, new CANSparkMax(Constants.FRONT_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
            speedMotors.set(Constants.FRONT_RIGHT, new CANSparkMax(Constants.FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
            speedMotors.set(Constants.BACK_LEFT, new CANSparkMax(Constants.BACK_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
            speedMotors.set(Constants.BACK_RIGHT, new CANSparkMax(Constants.BACK_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));

            pivotMotors.set(Constants.FRONT_LEFT, new CANSparkMax(Constants.FRONT_LEFT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
            pivotMotors.set(Constants.FRONT_RIGHT, new CANSparkMax(Constants.FRONT_RIGHT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
            pivotMotors.set(Constants.BACK_LEFT, new CANSparkMax(Constants.BACK_LEFT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
            pivotMotors.set(Constants.BACK_RIGHT, new CANSparkMax(Constants.BACK_RIGHT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));

            dutyCycles.set(Constants.FRONT_LEFT, new DutyCycle(new DigitalInput(Constants.FRONT_LEFT_ABSOLUTE_DIO_PORT)));
            dutyCycles.set(Constants.BACK_LEFT, new DutyCycle(new DigitalInput(Constants.BACK_LEFT_ABSOLUTE_DIO_PORT)));
            dutyCycles.set(Constants.BACK_RIGHT, new DutyCycle(new DigitalInput(Constants.BACK_RIGHT_ABSOLUTE_DIO_PORT)));
            dutyCycles.set(Constants.FRONT_RIGHT, new DutyCycle(new DigitalInput(Constants.FRONT_RIGHT_ABSOLUTE_DIO_PORT)));

            pivotMotors.forEach(m -> {
                // When we cut power, the motors should stop pivoting immediately;
                // otherwise, the slop will throw us off.
                m.setIdleMode(CANSparkMax.IdleMode.kBrake);

                // The conversion factor translates rotations of the pivot motor to
                // rotations of the swerve wheel.  Doing this allows us to pass
                // whole rotations into CANEncoder.setReference().
                //
                // TODO: Remove.  We aren't using the built-in encoders in the Spark MAX anymore.
                m.getEncoder().setPositionConversionFactor(1 / Constants.WHEEL_TURN_RATIO);
            });

            // Inverts drive motors based on reversal flags list
            for (int i = 0; i < 4; i++) {
                var m = speedMotors.get(i);
                m.setInverted(reversalFlags.get(i));
            }

        } catch (Throwable e) {
            System.out.printf("Exception thrown: %s", e.getMessage());
            e.printStackTrace();
        }

        this.pidControllers = new ArrayList<>();
        int i = 0;
        for (CANSparkMax pivotMotor : pivotMotors) {
            try {
                System.out.printf("about to get pidcontroller #%d\n", i++);
                ProfiledPIDController pidController = new ProfiledPIDController(P, I, D,
                    new TrapezoidProfile.Constraints(360  * 5, 360 * 8));

                // The pidController is supposed to minimize the deviation (error) between
                // an absolute angle, in degrees, and a setpoint coming from the human
                // driver's controller, also in degrees.  To minimize this angle, we need
                // to be aware that the range "wraps around" between 0 and 360 degrees.
                pidController.enableContinuousInput(-180, 180);

                // Set PID coefficients.
                pidController.setP(P);
                pidController.setI(I);
                pidController.setD(D);

                // Sets our iZone value.  This is extremely important, and warrants an explanation.
                //
                // - The Integral term (I) is used to get us exactly to our setpoint when the
                //   measurement is close enough.
                // - We use an integral value of 1.0, which is considered fairly high by field techs.
                // - High I values are subject to a phenemonen known as 'integral windup.'  Imagine having
                //   a setpoint of 50 degrees and a measured value of 49.95.  You're close!  You turn, and you
                //   overshoot.  You need to turn back.  Every time this happens, error accumulates.
                //   * On a smooth surface, or when the robot is suspended in the air with no friction, we easily
                //     resolve by reaching the setpoint quickly.
                //   * On higher-friction surfaces, it takes longer to get to the setpoint, and the error accumulates
                //     more quickly.  For us, it accumulated too quickly, leading to integral windup.
                // - The setIntegratorRange() function sets an integral zone ("iZone") so that when the integral
                //   value suggests a correction that is too large, we say "no thank you" and ignore it.
                pidController.setIntegratorRange(-2.0, 2.0);

                pidControllers.add(pidController);

            } catch (Exception e) {
                System.out.printf("Could not get pid controller: %s\n", e.getMessage());
                e.printStackTrace();
            }
        }

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        for (int j = 0; j < 4; j++) {
            String s = Constants.CORNER_NAME_ABBREVS[j];
            entries.set(j + 0, shuffleboardTab.add(String.format("%s delta", s), 0).getEntry());
            entries.set(j + 4, shuffleboardTab.add(String.format("%s angle", s), 0).getEntry());
            entries.set(j + 8, shuffleboardTab.add(String.format("%s goal", s), 0).getEntry());
            entries.set(j + 12, shuffleboardTab.add(String.format("%s speed", s), 0).getEntry());
        }
    }

    /**
     * Stores a new direction for the swerve modules to go in.
     *
     * <p>You can pass
     * these values into {@link #drive(SwerveModuleState[]) drive()} using
     * {@link #getGoalStates()}.</p>
     *
     * @param newGoalStates New shopping cart angles and speeds that you want.
     */
    public void setGoalStates(SwerveModuleState[] newGoalStates) {
        goalStates = newGoalStates;
    }

    /**
     * Returns the goal states that were previously set by {@link
     * #setGoalStates(SwerveModuleState[]) setGoalStates()} so that you can
     * pass those into {@link #drive(SwerveModuleState[]) drive()}.
     *
     * @return Returns the goal states that were passed into setGoalStates().
     */
    public SwerveModuleState[] getGoalStates() {
        return goalStates;
    }

    /**
     * Returns {@link SwerveModuleState SwerveModuleStates} where all four pivot wheels
     * are aligned at the given absolute angle, regardless of where the the wheels were
     * when the robot was turned on.
     *
     * All four modules will have a speed of 0.
     *
     * @param absoluteAngleDegrees The desired angle for all the swerve modules.
     *                             An angle of 0 points all modules forward, and is the
     *                             correct position for teleopInit().  Positive values rotate
     *                             clockwise and negative values rotate counterclockwise.
     * @return The states.
     */
    public SwerveModuleState[] reset(double absoluteAngleDegrees) {
        double absoluteAngleRadians = absoluteAngleDegrees * Math.PI/ 180;
        return new SwerveModuleState[] {
            new SwerveModuleState(0, new Rotation2d(absoluteAngleRadians)),
            new SwerveModuleState(0, new Rotation2d(absoluteAngleRadians)),
            new SwerveModuleState(0, new Rotation2d(absoluteAngleRadians)),
            new SwerveModuleState(0, new Rotation2d(absoluteAngleRadians))
        };
    }

    @Override
    public void drive(SwerveModuleState[] swerveModuleStates) {

        // Translates shopping cart speeds and angles into motion.
        for (int i = 0; i < swerveModuleStates.length; i++)  {

            // The absolute position of the swerve wheels.  We need the
            // displacement offsets in order to force the wheels into the
            // angles we actually want.
            double currentAbsoluteAngle = dutyCycles.get(i).getOutput() * 360;
            double displacementAngleDegrees = Constants.DISPLACEMENT_ANGLES[i];
            currentAbsoluteAngle = (currentAbsoluteAngle - (displacementAngleDegrees - 180)) % 360 ;
            entries.get(i + 4).setDouble(currentAbsoluteAngle);

            // Set the speed for the current drive motor.
            // ------------------------------------------

            // Translate speed from units of meters per second into a unitless
            // value between -1.0 and 1.0.
            SwerveModuleState state = swerveModuleStates[i];
            double speed = state.speedMetersPerSecond /
                Constants.ROBOT_MAXIMUM_SPEED_METERS_PER_SECOND;
            if (true) {
                // Make the current drive motor go vroom.
                speedMotors.get(i).set(speed);
            }
            entries.get(i + 12).setDouble(speedMotors.get(i).get());

            // Set angle for current pivot motor.
            // ----------------------------------

            // This is the angle coming from the shopping cart angles in the swerveModuleStates[]
            // argument.  The range is always [-180, 180].
            double rotations = state.angle.getDegrees() + 180.0;
            entries.get(i + 8).setDouble(rotations);

            // Tells us the distance between our desired angle from the controller and our current pivot motor angle, in degrees.
            double deltaDegrees = pidControllers.get(i).calculate(currentAbsoluteAngle, rotations);
            entries.get(i + 0).setDouble(deltaDegrees);

            if (!pidControllers.get(i).atSetpoint()) {
                // Percent of the distance we want to rotate relative to our desired degrees in this loop
                if (true) {
                    // Make the current pivot motor go vroom.
                    final double MAX_TURNING_RATE = 1.0;
                    pivotMotors.get(i).set((deltaDegrees / 180) * MAX_TURNING_RATE);
                }
            }
        }
    }
}
