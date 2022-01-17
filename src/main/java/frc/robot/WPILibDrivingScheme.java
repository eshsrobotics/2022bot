package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/**
 * Performs the translation from holonomic inputs into swerve module outputs in the
 * way WPILib thinks we're supposed to do it.
 */
public class WPILibDrivingScheme implements DrivingScheme {

    private SwerveDriveKinematics kinematics = null;
    private Gyro gyro = null;

    /**
     * It's not our long-term goal to have this driving scheme object own its
     * own kinematics or gyro objects; those properly belong to the wheel drive
     * subsystem.
     *
     * We supply a default constructor for now just to allow us to proceed with testing.
     */
    public WPILibDrivingScheme() {
        final double horizontal = Constants.WHEEL_BASE_WIDTH_METERS/2;
        final double vertical = Constants.WHEEL_BASE_LENGTH_METERS/2;
        kinematics = new SwerveDriveKinematics(
            new Translation2d(-horizontal, +vertical), // FRONT_LEFT
            new Translation2d(-horizontal, -vertical), // BACK_LEFT
            new Translation2d(+horizontal, -vertical), // BACK_RIGHT
            new Translation2d(+horizontal, +vertical)  // FRONT_RIGHT
        );
        gyro = new ADXRS450_Gyro();
    }

    /**
     * Constructs a drive scheme object using external objects.
     *
     * This is the way you're _supposed_ to construct these.
     * @param kinematics The {@link SwerveDriveKinematics} to use for translating
     * {@link ChassisSpeeds overall chassis speeds} into swerve wheel movement.
     */
    public WPILibDrivingScheme(SwerveDriveKinematics kinematics, Gyro gyro) {
        this.kinematics = kinematics;
        this.gyro = gyro;
    }

    /**
     * Given the user's holonomic inputs, returns the angles and speeds of all
     * four swerve modules.
     */
    @Override
    public SwerveModuleState[] convert(double frontBack, double leftRight, double rotation) {

        final double frontBackMetersPerSecond =
            frontBack * Constants.ROBOT_MAXIMUM_SPEED_METERS_PER_SECOND;
        final double leftRightMetersPerSecond =
            leftRight * Constants.ROBOT_MAXIMUM_SPEED_METERS_PER_SECOND;
        final double rotationRadiansPerSecond =
            rotation * Constants.ROBOT_MAXIMUM_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        ChassisSpeeds chassisSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(frontBackMetersPerSecond,
                                                  leftRightMetersPerSecond,
                                                  rotationRadiansPerSecond,
                                                  gyro.getRotation2d());

        return kinematics.toSwerveModuleStates(chassisSpeeds);
    }

}
