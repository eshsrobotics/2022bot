package frc.robot;

import java.util.ArrayList;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SparkMaxSwerveDriver implements SwerveDriver {
    private List<CANSparkMax> pivotMotors;
    private List<CANSparkMax> speedMotors;

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
     * Initializes this object.  We need access to {@link CANSparkMAX} objects
     * to do our dirty work, but since we should not _own_ them (that's
     * {@link SwerveDriveSubsystem SwerveDriveSubsystem's} job!), the caller
     * must pass them in at construction time.
     *
     * @param pivotMotors An list of four speed controllers that control the
     *                    angles of the swerve drive wheels.
     * @param speedMotors A list of four speed controllers that control the
     *                    speed of the swerve drive wheels.
     */
    public SparkMaxSwerveDriver(List<CANSparkMax> pivotMotors,
                                List<CANSparkMax> speedMotors) {
        this.pivotMotors = pivotMotors;
        this.speedMotors = speedMotors;
        this.pidControllers = new ArrayList<SparkMaxPIDController>();
        for (CANSparkMax pivotMotor : pivotMotors) {
            var pidController = pivotMotor.getPIDController();

            // Set PID coefficients.
            pidController.setP(0.1);
            pidController.setI(1e-4);
            pidController.setD(1);
            pidController.setIZone(0);
            pidController.setFF(0);
            pidController.setOutputRange(MIN_OUTPUT, MAX_OUTPUT);

            pidControllers.add(pidController);
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
