package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.driveSchemes.DrivingScheme;
import frc.robot.driveSchemes.WPILibDrivingScheme;
import frc.robot.drivers.SparkMaxSwerveDriver;

/**
 * Continuously updates the eight motors of the serve drive -- four pivot
 * motors for aiming and four drive motors -- using two sources of
 * {@link SwerveModuleState SwerveModuleStates}: human input (by
 * means of a {@link DrivingScheme}) and autonomous trajectories.
 *
 * Put succinctly, this class makes the robot go vroom.
 */
public class SwerveDriveSubsystem extends SubsystemBase {

    /**
     * Autonomous routine run time that allows us to taxi from the tarmak. 
     */
    private static final double AUTONOMOUS_RUN_DURATION_SEC = 3.0;

    /**
     * Measure the start time when we start autonomous.
     */
    private double autonomousStartTimeSeconds = 0.0;

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
    private SparkMaxSwerveDriver driver = null;

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

    private NetworkTableEntry changeSwerveAngleEntry = null;

    /**
     * We have a {@link SwerveDriveSubsystem#changeSwerveAngleEntry shuffleboard variable}
     * that can be used to force the swerve drive to point at a given angle.  But!  We
     * only want to use this value if the human actually changes it in the Shuffleboard
     * (since otherwise this debugging aid would have a negative impact on actual driving.)
     *
     * <p>To do this, we need to maintain some state: namely, the last value that we saw for
     * this ShuffleBoard entry.</p>
     */
    private double lastAngleFromShuffleboard = 0;

    /**
     * Initializes the drive and gyro. Whenever you start, the robot should
     * start with the front pointing away from the driver.
     *
     * We initializes all of the PID controllers here, so all of the PID
     * constants are set to the same value for now.
     * @param inputSubsystem Take human input and translates them into values
     *                       useful to us.
     */
    public SwerveDriveSubsystem(InputSubsystem inputSubsystem, Gyro gyro) {

        this.inputSubsystem = inputSubsystem;
        this.gyro = gyro;

        final double horizontal = Constants.WHEEL_BASE_WIDTH_METERS/2;
        final double vertical = Constants.WHEEL_BASE_LENGTH_METERS/2;
        kinematics = new SwerveDriveKinematics(
            // new Translation2d(-horizontal, +vertical), // FRONT_LEFT
            // new Translation2d(-horizontal, -vertical), // BACK_LEFT
            // new Translation2d(+horizontal, -vertical), // BACK_RIGHT
            // new Translation2d(+horizontal, +vertical)  // FRONT_RIGHT
            new Translation2d(-horizontal, +vertical), // FRONT_LEFT
            new Translation2d(+horizontal, +vertical),  // FRONT_RIGHT
            new Translation2d(+horizontal, -vertical), // BACK_RIGHT
            new Translation2d(-horizontal, -vertical) // BACK_LEFT


        );

        this.drivingScheme = new WPILibDrivingScheme(kinematics, gyro);

        // This currently blows up with a ClassNotFoundException at runtime on the robot,
        // long before teleop or autonomous even start.
        // this.driver = new SwerveLibDriver();
        this.driver = new SparkMaxSwerveDriver();

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Drivetrain");
        changeSwerveAngleEntry = shuffleboardTab.add("Set Swerve Angle", lastAngleFromShuffleboard).getEntry();
      }

    /**
     * Returns the {@link Gyro gyro} that this subsystem is using for
     * field-oriented swerve operations.
     */
    public Gyro getGyro() {
        return gyro;
    }

    /**
     * Continuously updates the swerve modules' speeds and angles based on autonomous state
     * and/or user input.
     */
    @Override
    public void periodic() {
        SwerveModuleState[] stateFromController = drivingScheme.convert(inputSubsystem.getFrontBack(),
                                                                        inputSubsystem.getLeftRight(),
                                                                        inputSubsystem.getRotation());

        double angleFromShuffleboard = changeSwerveAngleEntry.getDouble(0);
        if (!inputSubsystem.joysticksAtNeutral()) {

            // Drive based on human input only.
            driver.setGoalStates(stateFromController);

        } else if (angleFromShuffleboard != lastAngleFromShuffleboard) {

            // Change the pivot motor angles based on a shuffle board variable, but *only*
            // if that value changed.
            driver.setGoalStates(driver.reset(angleFromShuffleboard));
            lastAngleFromShuffleboard = angleFromShuffleboard;

        } else if (Timer.getFPGATimestamp() - autonomousStartTimeSeconds < AUTONOMOUS_RUN_DURATION_SEC) {

            // We are in the middle of an autonomous run.  The only thing we have to do in here is
            // not stop!
            
        } else {

            // We are not doing a debug change or autonomous, so the motors
            // should just stop.
            SwerveModuleState[] stopState = new SwerveModuleState[4];
            for (int i = 0; i < 4; ++i) {
                stopState[i] = new SwerveModuleState(0, driver.getGoalStates()[i].angle);
            }
            driver.setGoalStates(stopState);
        }

        driver.drive(driver.getGoalStates());
    }

    /**
     * Helper function for {@link frc.robot.RobotContainer#zeroPosition RobotContainer.zeroPosition()}.
     * From the start of deploying code, sets the swerve wheels forward to the initial postion that being 0
     * degrees.
     */
    public void initialPosition() {
        driver.setGoalStates(driver.reset(0));
    }

    /**
     * Starts the autonomous routine for a set amount of time.
     */
    public void startAutonomousRun() {
        autonomousStartTimeSeconds = Timer.getFPGATimestamp();
        SwerveModuleState[] states = drivingScheme.convert(-0.5, 0, 0);
        driver.setGoalStates(states);
    }
}
