package frc.robot.drivers;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
    private List<PIDController> pidControllers;

    /**
     * Converts inputs from PWM into dutyCycles between zero and one.
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
    private final double P = 1.0;
    private final double I = 1.0;
    private final double D = 0.01;

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
            // reversalFlags.set(Constants.FRONT_LEFT, true);
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

        this.pidControllers = new ArrayList<PIDController>();
        int i = 0;
        for (CANSparkMax pivotMotor : pivotMotors) {
            try {
                System.out.printf("about to get pidcontroller #%d\n", i++);
                PIDController pidController = new PIDController(P, I, D);

                // The pidController is supposed to minimize the deviation (error) between
                // an aboslute angle, in degrees, and a setpoint coming from the human
                // driver's controller, also in degrees.  To minimize this angle, we need
                // to be aware that the range "wraps around" between 0 and 360 degrees.
                pidController.enableContinuousInput(-180, 180);

                // Set PID coefficients.
                pidController.setP(P);
                pidController.setI(I);
                pidController.setD(D);


                // Integrated encoder is the feedback device the controller uses by default.
                // pidController.setFeedbackDevice(pivotMotor.getEncoder());

                pidControllers.add(pidController);
            } catch (Exception e) {
                System.out.printf("Could not get pid controller: %s\n", e.getMessage());
                e.printStackTrace();
            }
        }

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        // shuffleboardTab.addNumber("fLAltEnc", () -> pivotMotors.get(Constants.FRONT_LEFT).getAlternateEncoder(Constants.SRX_MAG_ENCODER_CLICKS_PER_REVOLUTION).getPosition());
        // shuffleboardTab.addNumber("fRAltEnc", () -> pivotMotors.get(Constants.FRONT_RIGHT).getAlternateEncoder(Constants.SRX_MAG_ENCODER_CLICKS_PER_REVOLUTION).getPosition());
        // shuffleboardTab.addNumber("bLAltEnc", () -> pivotMotors.get(Constants.BACK_LEFT).getAlternateEncoder(Constants.SRX_MAG_ENCODER_CLICKS_PER_REVOLUTION).getPosition());
        // shuffleboardTab.addNumber("bRAltEnc", () -> pivotMotors.get(Constants.BACK_RIGHT).getAlternateEncoder(Constants.SRX_MAG_ENCODER_CLICKS_PER_REVOLUTION).getPosition());

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
     * You can pass these values into drive() using getGoalStates().
     *
     * @param newGoalStates New shopping cart angles and speeds that you want.
     */
    public void setGoalStates(SwerveModuleState[] newGoalStates) {
        goalStates = newGoalStates;
    }

    /**
     * Returns the goal states that were previously set by setGoalStates()
     * so that you can pass those into drive().
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
     * @param absoluteAngleDegrees The desired angle for all the swerve modules.
     *                             An angle of 0 points all modules forward, and is the
     *                             correct position for teleopInit().  Positive values rotate
     *                             clockwise and negative values rotate counterclockwise.
     * @return The states.
     */
    public SwerveModuleState[] reset(double absoluteAngleDegrees) {
        double absoluteAngleRadians = absoluteAngleDegrees * Math.PI/ 180;
        return new SwerveModuleState[] {
            new SwerveModuleState(absoluteAngleDegrees,
                                  new Rotation2d(absoluteAngleRadians)),
            new SwerveModuleState(absoluteAngleDegrees,
                                  new Rotation2d(absoluteAngleRadians)),
            new SwerveModuleState(absoluteAngleDegrees,
                                  new Rotation2d(absoluteAngleRadians)),
            new SwerveModuleState(absoluteAngleDegrees,
                                  new Rotation2d(absoluteAngleRadians))
        };
    }

    @Override
    public void drive(SwerveModuleState[] swerveModuleStates) {

        // Translates shopping cart speeds and angles into motion.
        for (int i = 0; i < swerveModuleStates.length; i++)  {

            // Set the speed for the current drive motor.
            // ------------------------------------------

            // Translate speed from units of meters per second into a unitless
            // value between -1.0 and 1.0.
            double speed = swerveModuleStates[i].speedMetersPerSecond /
                Constants.ROBOT_MAXIMUM_SPEED_METERS_PER_SECOND;
            speedMotors.get(i).set(speed);
            entries.get(i + 12).setDouble(speedMotors.get(i).get());

            // Set angle for current pivot motor.
            // ----------------------------------

            // This is the angle coming from the shopping cart angles in the swerveModuleStates[]
            // argument.  The range is always [-180, 180].
            double rotations = swerveModuleStates[i].angle.getDegrees() + 180.0;
            entries.get(i + 8).setDouble(rotations);

            // The absolute position of the swerve wheels.  We need the displacement offsets
            // in order to force the wheels into the angles we actually want.
            double currentAbsoluteAngle = dutyCycles.get(i).getOutput() * 360;
            double displacementAngleDegrees = Constants.DISPLACEMENT_ANGLES[i];
            currentAbsoluteAngle = (currentAbsoluteAngle - (displacementAngleDegrees - 180)) % 360 ;
            entries.get(i + 4).setDouble(currentAbsoluteAngle);

            // Tells us the distance between our desired angle from the controller and our current pivot motor angle, in degrees.
            double deltaDegrees = pidControllers.get(i).calculate(currentAbsoluteAngle, rotations);
            entries.get(i + 0).setDouble(deltaDegrees);

            if (!pidControllers.get(i).atSetpoint()) {
                // Percent of the distance we want to rotate relative to our desired degrees in this loop
                final double MAX_TURNING_RATE = 1.0;
               pivotMotors.get(i).set((deltaDegrees / 180) * MAX_TURNING_RATE);
            }
        }
    }
}
