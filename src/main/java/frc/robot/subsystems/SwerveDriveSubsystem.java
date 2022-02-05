package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.driveSchemes.DrivingScheme;
import frc.robot.driveSchemes.WPILibDrivingScheme;
import frc.robot.drivers.SparkMaxSwerveDriver;
import frc.robot.drivers.SwerveDriver;
import frc.robot.drivers.SwerveLibDriver;
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
        
        final double horizontal = Constants.WHEEL_BASE_WIDTH_METERS/2;
        final double vertical = Constants.WHEEL_BASE_LENGTH_METERS/2;
        kinematics = new SwerveDriveKinematics(
            new Translation2d(-horizontal, +vertical), // FRONT_LEFT
            new Translation2d(-horizontal, -vertical), // BACK_LEFT
            new Translation2d(+horizontal, -vertical), // BACK_RIGHT
            new Translation2d(+horizontal, +vertical)  // FRONT_RIGHT
        );

        this.drivingScheme = new WPILibDrivingScheme(kinematics, gyro);

        // This currently blows up with a ClassNotFoundException at runtime on the robot,
        // long before teleop or autonomous even start.
        // this.driver = new SwerveLibDriver();
        this.driver = new SparkMaxSwerveDriver();

    }

    /**
     * Continuously updates the swerve modules' speeds and angles based on autonomous state
     * and/or user input.
     */
    @Override
    public void periodic() {
        SwerveModuleState[] currentState = drivingScheme.convert(inputSubsystem.getFrontBack(), 
                                                                 inputSubsystem.getLeftRight(), 
                                                                 inputSubsystem.getRotation());
        driver.drive(currentState);
        
    }
}
