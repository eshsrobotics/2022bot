package frc.robot.driveSchemes;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;

/**
 * Performs the translation from holonomic inputs into swerve module outputs in the
 * way WPILib thinks we're supposed to do it.
 */
public class WPILibDrivingScheme implements DrivingScheme {

    private SwerveDriveKinematics kinematics = null;
    private Gyro gyro = null;

    private NetworkTableEntry[] swerveEntries = new NetworkTableEntry[] {
        null, null, null, null, // Pivot angles (FL, FR, BR, BL)
        null, null, null, null, // Speeds (FL, FR, BR, BL)
        null, null, null        // ChassisSpeeds (x, y, rotation)
    };

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
        initializeShuffleboard();
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
        initializeShuffleboard();
    }

    /**
     * Lays out the eight widgets in the ShuffleBoard's "DrivingScheme" tab
     * needed to fully represent a single {@link SwerveModuleState}.
     */
    private void initializeShuffleboard() {
        var tab = Shuffleboard.getTab("DrivingScheme");
        var layout = tab.getLayout("Swerve Module States", BuiltInLayouts.kGrid)
            .withPosition(4, 0)
            .withSize(4, 6);
        swerveEntries[Constants.FRONT_LEFT + 0] = layout.add("FL angle", 0)
            .withPosition(0, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGyro)
            .getEntry();
        swerveEntries[Constants.FRONT_LEFT + 4] = layout.add("FL speed", 0)
            .withPosition(0, 2)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kDial)
            .getEntry();
        swerveEntries[Constants.FRONT_RIGHT + 0] = layout.add("FR angle", 0)
            .withPosition(2, 0)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGyro)
            .getEntry();
        swerveEntries[Constants.FRONT_RIGHT + 4] = layout.add("FR speed", 0)
            .withPosition(2, 2)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kDial)
            .getEntry();
        swerveEntries[Constants.BACK_LEFT + 0] = layout.add("BL angle", 0)
            .withPosition(0, 3)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGyro)
            .getEntry();
        swerveEntries[Constants.BACK_LEFT + 4] = layout.add("BL speed", 0)
            .withPosition(0, 5)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kDial)
            .getEntry();
        swerveEntries[Constants.BACK_RIGHT + 0] = layout.add("BR angle", 0)
            .withPosition(2, 3)
            .withSize(2, 2)
            .withWidget(BuiltInWidgets.kGyro)
            .getEntry();
        swerveEntries[Constants.BACK_RIGHT + 4] = layout.add("BR speed", 0)
            .withPosition(2, 5)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kDial)
            .getEntry();

        var chassisSpeedsLayout = tab.getLayout("ChassisSpeeds", BuiltInLayouts.kList)
            .withPosition(8, 0)
            .withSize(2, 6);
        swerveEntries[8] = chassisSpeedsLayout.add("Vx", 0)
            .withWidget(BuiltInWidgets.kDial)
            .getEntry();
        swerveEntries[9] = chassisSpeedsLayout.add("Vy", 0)
            .withWidget(BuiltInWidgets.kDial)
            .getEntry();
        swerveEntries[10] = chassisSpeedsLayout.add("Vtheta", 0)
            .withWidget(BuiltInWidgets.kGyro)
            .getEntry();


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
                                                  -leftRightMetersPerSecond,
                                                  rotationRadiansPerSecond,
                                                  gyro.getRotation2d());
        swerveEntries[8].setDouble(chassisSpeeds.vxMetersPerSecond);
        swerveEntries[9].setDouble(chassisSpeeds.vyMetersPerSecond);
        swerveEntries[10].setDouble(chassisSpeeds.omegaRadiansPerSecond);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        for (int i = 0; i < 4; i++) {
            states[i].angle = new Rotation2d(-states[i].angle.getRadians());
            // Update the shuffleboard (one way, read-only.)
            swerveEntries[i + 0].setDouble(states[i].angle.getDegrees());
            swerveEntries[i + 4].setDouble(states[i].speedMetersPerSecond);
        }
        return states;
    }

}
