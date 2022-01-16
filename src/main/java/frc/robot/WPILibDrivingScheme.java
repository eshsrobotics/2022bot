package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class WPILibDrivingScheme implements DrivingScheme {
    private SwerveDriveKinematics kinematics = null;
    private Gyro gyro = null;

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

    public WPILibDrivingScheme(SwerveDriveKinematics kinematics, Gyro gyro) {
        this.kinematics = kinematics;
        this.gyro = gyro;
    }

    @Override
    public SwerveModuleState[] convert(double frontBack, double leftRight, double rotation) {

        final double frontBackMetersPerSecond = frontBack * Constants.ROBOT_MAXIMUM_SPEED_METERS_PER_SECOND;
        final double leftRightMetersPerSecond = leftRight * Constants.ROBOT_MAXIMUM_SPEED_METERS_PER_SECOND;
        final double rotationRadiansPerSecond = rotation * Constants.ROBOT_MAXIMUM_ANGULAR_VELOCITY_RADIANS_PER_SECOND;

        ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(frontBackMetersPerSecond, leftRightMetersPerSecond, rotationRadiansPerSecond, gyro.getRotation2d());

        return kinematics.toSwerveModuleStates(chassisSpeeds);
    }
    
}
