import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DrivingScheme;
import frc.robot.SwerveDriver;
import frc.robot.WPILibDrivingScheme;
import frc.robot.subsystems.InputSubsystem;

/**
 * Continuously updates the eight motors of the serve drive -- four pivot
 * motors for aiming and four drive motors -- using two sources of
 * {@link SwerveModuleState SwerveModuleStates}: human input (by
 * means of a {@link DriveScheme}) and autonomous trajectories.
 *
 * Put succinctly, this class makes the robot go vroom.
 */
public class SwerveDriveSubsystem extends SubsystemBase {

    /**
     * Translates human input into three numbers: rotate, left/right, and
     * forward/back.
     */
    private InputSubsystem inputSubsystem = null;

    /**
     * Translates the 3 human inputs to 4 swerve module states.
     */
    private DrivingScheme drivingScheme = null;

    /**
     * Converts an array of four swerve module states into motion.
     */
    private SwerveDriver driver = null;

    /**
     * Tells what degrees the robot is in relative to the field
     */
    private Gyro gyro = null;

    /**
     * Utility class that we use to generate the
     * {@link SwerveModuleState ServeModuleStates}.  It also helps with the
     * field-oriented swerve calculations.
     */
    private SwerveDriveKinematics kinematics = null;

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
     * Initializes the drive and gyro. Whenever you start, the robot should
     * start with the front pointing away from the driver.
     *
     * We initializes all of the PID controllers here, so all of the PID
     * constants are set to the same value for now.
     * @param inputSubsystem Take human input and translates them into values
     *                       useful to us.
     */
    public SwerveDriveSubsystem(InputSubsystem inputSubsystem) {

        this.inputSubsystem = inputSubsystem;
        this.gyro = new ADXRS450_Gyro();
        this.drivingScheme = new WPILibDrivingScheme(kinematics, gyro);
        this.driver = new SwerveLibDriver();
        

        final double horizontal = Constants.WHEEL_BASE_WIDTH_METERS/2;
        final double vertical = Constants.WHEEL_BASE_LENGTH_METERS/2;
        kinematics = new SwerveDriveKinematics(
            new Translation2d(-horizontal, +vertical), // FRONT_LEFT
            new Translation2d(-horizontal, -vertical), // BACK_LEFT
            new Translation2d(+horizontal, -vertical), // BACK_RIGHT
            new Translation2d(+horizontal, +vertical)  // FRONT_RIGHT
        );


        // Initialize the swerve motors (pivot and speed.)
        reversalFlags = new ArrayList<Boolean>();
        pivotMotors = new ArrayList<CANSparkMax>();
        speedMotors = new ArrayList<CANSparkMax>();
        Collections.addAll(pivotMotors, new CANSparkMax[] { null, null, null, null});
        Collections.addAll(speedMotors, new CANSparkMax [] { null, null, null, null});
        Collections.addAll(reversalFlags, new Boolean[] { true, true, false, false});
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

    }
}
