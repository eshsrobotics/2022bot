package frc.robot.drivers;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;

public class SparkMaxSwerveDriver implements SwerveDriver {
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
    private List<SparkMaxPIDController> pidControllers;

    private final double P = 0.1;
    private final double I = 1e-4;
    private final double D = 1;
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
            Collections.addAll(pivotMotors, new CANSparkMax[] { null, null, null, null });
            Collections.addAll(speedMotors, new CANSparkMax [] { null, null, null, null });
            Collections.addAll(reversalFlags, new Boolean[] { true, true, false, false });

            speedMotors.set(Constants.FRONT_LEFT, new CANSparkMax(Constants.FRONT_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
            speedMotors.set(Constants.FRONT_RIGHT, new CANSparkMax(Constants.FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
            speedMotors.set(Constants.BACK_LEFT, new CANSparkMax(Constants.BACK_LEFT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));
            speedMotors.set(Constants.BACK_RIGHT, new CANSparkMax(Constants.BACK_RIGHT_DRIVE_MOTOR_CAN_ID, MotorType.kBrushless));

            pivotMotors.set(Constants.FRONT_LEFT, new CANSparkMax(Constants.FRONT_LEFT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
            pivotMotors.set(Constants.FRONT_RIGHT, new CANSparkMax(Constants.FRONT_RIGHT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
            pivotMotors.set(Constants.BACK_LEFT, new CANSparkMax(Constants.BACK_LEFT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));
            pivotMotors.set(Constants.BACK_RIGHT, new CANSparkMax(Constants.BACK_RIGHT_TURN_MOTOR_CAN_ID, MotorType.kBrushless));

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

        this.pidControllers = new ArrayList<SparkMaxPIDController>();
        int i = 0;
        for (CANSparkMax pivotMotor : pivotMotors) {
            try {
                System.out.printf("about to get pidcontroller #%d\n", i++);
                SparkMaxPIDController pidController = pivotMotor.getPIDController();

                // Set PID coefficients.
                pidController.setP(0.1);
                pidController.setI(1e-4);
                pidController.setD(1);
                pidController.setIZone(0);
                pidController.setFF(0);
                pidController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

                pidControllers.add(pidController);
            } catch (Exception e) {
                System.out.printf("Could not get pid controller: %s\n", e.getMessage());
                e.printStackTrace();
            }
        }
    }

    @Override
    public void drive(SwerveModuleState[] swerveModuleStates) {
        // Translates shopping cart speeds and angles into motion.
        for  (int i = 0; i < swerveModuleStates.length; i++)  {

            // Translate speed from units of meters per second into a unitless
            // value between -1.0 and 1.0.
            double speed = swerveModuleStates[i].speedMetersPerSecond /
                Constants.ROBOT_MAXIMUM_SPEED_METERS_PER_SECOND;
            speedMotors.get(i).set(speed);

            // Set angle for current pivot motor.
            double rotations = swerveModuleStates[i].angle.getDegrees() / 360;
            pidControllers.get(i).setReference(rotations,
                                               CANSparkMax.ControlType.kPosition);
        }
    }
}
