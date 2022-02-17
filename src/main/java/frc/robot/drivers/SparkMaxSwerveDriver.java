package frc.robot.drivers;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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
    private List<NetworkTableEntry> pivotDeltaEntries;

     /**
     * Controls the speed of the four swerve wheels.
     */
    private List<CANSparkMax> speedMotors;

    /**
     * This is an array of 4 boolean values, one for each of the 4 speed motors.
     * Whenever the reversal flag is true for a motor, then we will reverse the
     * motor's direction in software.
     */
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

    private final double P = 0.1;
    private final double I = 1e-4;
    private final double D = 0.05;
    private final double Iz = 0;
    private final double FF = 0;
    private final double MAX_OUTPUT = 1;
    private final double MIN_OUTPUT = -1;

    /**
     * Initializes all of the {@link CANSparkMax} motor and PID controllers.
     */
    public SparkMaxSwerveDriver() {
        // Initialize the swerve motors (pivot and speed.)
        try {
            reversalFlags = new ArrayList<Boolean>();
            pivotMotors = new ArrayList<CANSparkMax>();
            speedMotors = new ArrayList<CANSparkMax>();
            dutyCycles = new ArrayList<DutyCycle>();
            pivotDeltaEntries = new ArrayList<NetworkTableEntry>();
            Collections.addAll(pivotMotors, new CANSparkMax[] { null, null, null, null });
            Collections.addAll(speedMotors, new CANSparkMax [] { null, null, null, null });
            Collections.addAll(reversalFlags, new Boolean[] { true, true, false, false });
            Collections.addAll(dutyCycles, new DutyCycle[] { null, null, null, null});
            Collections.addAll(pivotDeltaEntries, new NetworkTableEntry[4]);

            speedMotors.set(Constants.FRONT_LEFT, new CANSparkMax(Constants.FRONT_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
            speedMotors.set(Constants.FRONT_RIGHT, new CANSparkMax(Constants.FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
            speedMotors.set(Constants.BACK_LEFT, new CANSparkMax(Constants.BACK_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
            speedMotors.set(Constants.BACK_RIGHT, new CANSparkMax(Constants.BACK_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));

            pivotMotors.set(Constants.FRONT_LEFT, new CANSparkMax(Constants.FRONT_LEFT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
            pivotMotors.set(Constants.FRONT_RIGHT, new CANSparkMax(Constants.FRONT_RIGHT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
            pivotMotors.set(Constants.BACK_LEFT, new CANSparkMax(Constants.BACK_LEFT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
            pivotMotors.set(Constants.BACK_RIGHT, new CANSparkMax(Constants.BACK_RIGHT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
            
            dutyCycles.set(Constants.FRONT_LEFT, new DutyCycle(new DigitalInput(Constants.FRONT_LEFT_ABSOLUTE_PWM_PORT)));
            dutyCycles.set(Constants.BACK_LEFT, new DutyCycle(new DigitalInput(Constants.BACK_LEFT_ABSOLUTE_PWM_PORT)));
            dutyCycles.set(Constants.BACK_RIGHT, new DutyCycle(new DigitalInput(Constants.BACK_RIGHT_ABSOLUTE_PWM_PORT)));
            dutyCycles.set(Constants.FRONT_RIGHT, new DutyCycle(new DigitalInput(Constants.FRONT_RIGHT_ABSOLUTE_PWM_PORT)));
            
            pivotMotors.forEach(m -> {
                // When we cut power, the motors should stop pivoting immediately;
                // otherwise, the slop will throw us off.
                m.setIdleMode(CANSparkMax.IdleMode.kBrake);

                // The conversion factor translates rotations of the pivot motor to
                // rotations of the swerve wheel.  Doing this allows us to pass
                // whole rotations into CANEncoder.setReference().
                m.getEncoder().setPositionConversionFactor(1 / Constants.WHEEL_TURN_RATIO);
            });

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
                // pidController.setIZone(Iz);
                // pidController.setFF(FF);
                // pidController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

                pidControllers.add(pidController);
            } catch (Exception e) {
                System.out.printf("Could not get pid controller: %s\n", e.getMessage());
                e.printStackTrace();
            }
        }
        
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        // shuffleboardTab.addNumber("fLDefEnc", () -> pivotMotors.get(Constants.FRONT_LEFT).getEncoder().getPosition());
        // shuffleboardTab.addNumber("fRDefEnc", () -> pivotMotors.get(Constants.FRONT_RIGHT).getEncoder().getPosition());
        // shuffleboardTab.addNumber("bLDefEnc", () -> pivotMotors.get(Constants.BACK_LEFT).getEncoder().getPosition());
        // shuffleboardTab.addNumber("bRDefEnc", () -> pivotMotors.get(Constants.BACK_RIGHT).getEncoder().getPosition());
        
        // shuffleboardTab.addNumber("fLAltEnc", () -> pivotMotors.get(Constants.FRONT_LEFT).getAlternateEncoder(Constants.SRX_MAG_ENCODER_CLICKS_PER_REVOLUTION).getPosition());
        // shuffleboardTab.addNumber("fRAltEnc", () -> pivotMotors.get(Constants.FRONT_RIGHT).getAlternateEncoder(Constants.SRX_MAG_ENCODER_CLICKS_PER_REVOLUTION).getPosition());
        // shuffleboardTab.addNumber("bLAltEnc", () -> pivotMotors.get(Constants.BACK_LEFT).getAlternateEncoder(Constants.SRX_MAG_ENCODER_CLICKS_PER_REVOLUTION).getPosition());
        // shuffleboardTab.addNumber("bRAltEnc", () -> pivotMotors.get(Constants.BACK_RIGHT).getAlternateEncoder(Constants.SRX_MAG_ENCODER_CLICKS_PER_REVOLUTION).getPosition());
        
        
        pivotDeltaEntries.set(Constants.FRONT_LEFT, shuffleboardTab.add("FL delta", 0).getEntry());
        pivotDeltaEntries.set(Constants.BACK_LEFT, shuffleboardTab.add("BL delta", 0).getEntry());
        pivotDeltaEntries.set(Constants.BACK_RIGHT, shuffleboardTab.add("BR delta", 0).getEntry());
        pivotDeltaEntries.set(Constants.FRONT_RIGHT, shuffleboardTab.add("FR delta", 0).getEntry());

    }

    @Override
    public void drive(SwerveModuleState[] swerveModuleStates) {
        // Translates shopping cart speeds and angles into motion.
        for  (int i = 0; i < swerveModuleStates.length; i++)  {

            // Translate speed from units of meters per second into a unitless
            // value between -1.0 and 1.0.
            double speed = swerveModuleStates[i].speedMetersPerSecond /
                Constants.ROBOT_MAXIMUM_SPEED_METERS_PER_SECOND;
            // speedMotors.get(i).set(speed);

            // Set angle for current pivot motor.
            // TODO: What happens if angle is NEGATIVE?
            double rotations = swerveModuleStates[i].angle.getDegrees() % 360.0;

            // Value is absolute because it does not change on robot power off
            double currentAbsoluteAngle = dutyCycles.get(i).getOutput() * 360;

            // Tells us the distance between our desired angle from the controller and our current pivot motor angle, in degrees.
            double deltaDegrees = pidControllers.get(i).calculate(currentAbsoluteAngle, rotations);

            // pivotDeltaEntries.get(i).setDouble(deltaDegrees);
            pivotDeltaEntries.get(i).setDouble(deltaDegrees);

            if (!pidControllers.get(i).atSetpoint()) {
                // Percent of the distance we want to rotate relative to our desired degrees in this loop
                final double MAX_TURNING_RATE = 1.0;
                pivotMotors.get(i).set(Math.signum(deltaDegrees) * (deltaDegrees / 360) * MAX_TURNING_RATE);
            }
        }
    }
}
