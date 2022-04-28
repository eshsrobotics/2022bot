package frc.robot.subsystems;

import java.util.HashMap;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants;

/**
 * Our robot supports multiple input methods (dual joysticks, XBox-compatible controllers, and even keyboard
 * control via networktablesinput.)  No matter how input comes from the human, in the end, we only have three
 * numbers to deal with:
 *
 * - The translation vector
 *   * Horizontal component (-1.0 to 1.0, positive values point away from the driver)
 *   * Vertical component (-1.0 to 1.0, positive values point to the driver's right)
 * - The rotation value (-1.0 to 1.0, positive values rotate clockwise)
 *
 * This class's purpose is to translate those inputs into those three numbers so that the drive subsystem
 * can use them during teleop.
 *
 * @see {@link frc.robot.driveSchemes.DrivingScheme DrivingScheme}, which uses these three values
 * to determine the direction and speed of the four swerve modules
 */
public class InputSubsystem extends SubsystemBase {

    private final static int DRIVE_CONTROLLER_INDEX = 0;
    private final static int AUXILIARY_CONTROLLER_INDEX = 1;

    /**
     * Our control scheme this year calls for not one, but <em>two</em>
     * XBox controllers: one for driving and normal operation, and one for
     * climbing and manual overrides.  This array, therefore, always has exactly
     * two elements: the drive at {@link #DRIVE_CONTROLLER_INDEX index 0} and
     * the auxiliary at {@link #AUXILIARY_CONTROLLER_INDEX index 1}.
     */
    private XboxController[] controllers =  null;

    private double frontBack = 0;
    private double leftRight = 0;
    private double rotation = 0;

    /**
     * A button that triggers a manual release, to the shooter, of a ball
     * captured by the indexer.  Nominally the release is automatic upon finding
     * a vision solution.
     */
    private Button fireButton_ = null;


    /**
     * A button that tracks whether or not the human drivers (well, really,
     * the human holding the auxiliary controller) wishes to manually override
     * the robot's behavior.
     */
    private Button manualOverrideButton_ = null;


    private Button hoodUpButton_ = null;
    private Button hoodDownButton_ = null;
    private Button turntableLeftButton_ = null;
    private Button turntableRightButton_ = null;
    private Button intakeToggleTestButton_ = null;
    private Button intakeReverseButton_ = null;
    private Button intakeDeployToggleButton_ = null;
    private Button climbUpButton_ = null;
    private Button climbDownButton_ = null;
    private Button rumbleDriveButton_ = null;
    private Button readShuffleboardButton_ = null;
    private Button resetGyroButton_ = null;
    private Button shooterFasterButton_ = null;
    private Button shooterSlowerButton_ = null;

    /**
     * Initializes this object and determines which input methods are usable.
     */
    public InputSubsystem() {
        controllers = new XboxController[2];
        assignControllersSimplistically();

        // Primary controller inputs.

        intakeToggleTestButton_ = new Button(() -> {
            return controllers[DRIVE_CONTROLLER_INDEX].getBButton();
        });
        intakeReverseButton_ = new Button (() -> {
            return controllers[DRIVE_CONTROLLER_INDEX].getXButton();
        });
        intakeDeployToggleButton_ = new Button (() -> {
            return controllers[DRIVE_CONTROLLER_INDEX].getAButton();
        });
        rumbleDriveButton_ = new Button (() -> {
            return controllers[DRIVE_CONTROLLER_INDEX].getRightStickButton();
        });

        // Make the Drive Controller rumble for a couple of seconds every time
        // you press R3 (i.e., when you treat the right joystick as a button and
        // press it down).
        rumbleDriveButton_.whenPressed(() -> {
            controllers[DRIVE_CONTROLLER_INDEX].setRumble(RumbleType.kLeftRumble, 1);
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                // We don't care if this happens.
            } finally {
                controllers[DRIVE_CONTROLLER_INDEX].setRumble(RumbleType.kLeftRumble, 0);
            }
        });

        // Auxiliary controller inputs.  Think of this as being for gunnery.

        fireButton_ = new Button(() -> {
            //return controller.getRightBumper();
            return controllers[AUXILIARY_CONTROLLER_INDEX].getRightTriggerAxis() > 0;
        });
        hoodUpButton_ = new Button(() -> {
            // Pressing UP on the D-Pad.
            int dpadAngle = controllers[AUXILIARY_CONTROLLER_INDEX].getPOV();
            return (dpadAngle >= 315 || dpadAngle <= 45);
        });
        hoodDownButton_ = new Button(()->{
            // Pressing DOWN on the D-Pad.
            int dpadAngle = controllers[AUXILIARY_CONTROLLER_INDEX].getPOV();
            return (dpadAngle >= 135 && dpadAngle <= 225);
        });
        shooterFasterButton_ = new Button(() -> {
            // Pressing RIGHT on the D-Pad.
            int dpadAngle = controllers[AUXILIARY_CONTROLLER_INDEX].getPOV();
            return (dpadAngle >= 45 && dpadAngle <= 135);
        });
        shooterSlowerButton_ = new Button(() -> {
            // Pressing LEFT on the D-Pad.
            int dpadAngle = controllers[AUXILIARY_CONTROLLER_INDEX].getPOV();
            return (dpadAngle >= 225 && dpadAngle <= 315);
        });
        turntableLeftButton_ = new Button(() -> {
            return (controllers[AUXILIARY_CONTROLLER_INDEX].getLeftBumper());
        });
        turntableRightButton_ = new Button(() -> {
            return (controllers[AUXILIARY_CONTROLLER_INDEX].getRightBumper());
        });
        climbDownButton_ = new Button (() -> {
            return controllers[AUXILIARY_CONTROLLER_INDEX].getBButton();
        });
        climbUpButton_ = new Button (() -> {
            return controllers[AUXILIARY_CONTROLLER_INDEX].getAButton();
        });
        manualOverrideButton_ = new Button (() -> {
            final int PRO_CONTROLLER_LEFT_TRIGGER_BUTTON = 7;
            final int PRO_CONTROLLER_RIGHT_TRIGGER_BUTTON = 8;
            return controllers[AUXILIARY_CONTROLLER_INDEX].getLeftTriggerAxis() > 0 ||
                   controllers[AUXILIARY_CONTROLLER_INDEX].getRawButton(PRO_CONTROLLER_LEFT_TRIGGER_BUTTON);
        });
        readShuffleboardButton_ = new Button(() ->{
            return controllers[AUXILIARY_CONTROLLER_INDEX].getYButton();
        });
        resetGyroButton_ = new Button(() -> {
            return controllers[DRIVE_CONTROLLER_INDEX].getYButton();
        });

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("InputSubsystem");
        shuffleboardTab.addNumber("frontBack", () -> frontBack);
        shuffleboardTab.addNumber("leftRight", () -> leftRight);
        shuffleboardTab.addNumber("rotation", () -> rotation);
        shuffleboardTab.addBoolean("turntableLeft", () -> turntableLeftButton_.get());
        shuffleboardTab.addBoolean("turntableRight", () -> turntableRightButton_.get());
        shuffleboardTab.addBoolean("manualOverride", () -> manualOverrideButton_.get());
    }

    /**
     * Assigns roles for controllers that are plugged in or connected via Bluetooth.
     *
     * The first controller this function sees in the virtual ports will be the drive controller.
     * The second controller this function sees will be the auxiliary controller.
     *
     * Behavior is undefined if you have less than two controllers plugged into the system, or
     * if one of the controllers is not an XBox controller.
     */
    public void assignControllersSimplistically() {
        final int NUM_VIRTUAL_PORTS = 6;
        for (int i = 0; i < NUM_VIRTUAL_PORTS; ++i) {
            XboxController controller = new XboxController(i);
            if (controller.isConnected()) {
                if (controllers[0] == null) {
                    controllers[0] = controller;
                } else if (controllers[1] == null) {
                    controllers[1] = controller;
                }
            }
        }

        if (controllers[0] == null) {
            System.out.printf("WARNING: No controllers assigned.  Are they all asleep?");
        // } else if (controllers[0] == controllers[1]) {
            //System.out.printf("WARNING: Driver and Auxiliary controller are the same.  Is there only one controller plugged in?");
        } else {
            controllers[1] = controllers[0];
            String driverName = (controllers[0] == null ? "(null)" : controllers[0].getName());
            String auxName = (controllers[1] == null ? "(null)" : controllers[1].getName());
            int driverId = controllers[0].hashCode();
            int auxId = controllers[1].hashCode();
            System.out.printf("Summary:\n  Drive Controller : %s (%d)\n  Auxiliary Controller : %s (%d)\n",
                driverName, driverId,
                auxName, auxId);
        }
    }

    /**
     * Returns the desired movement toward (negative) or away (positive) from the driver.
     */
    public double getFrontBack() {
        return -frontBack;
    }

    /**
     * Returns the desired movement to the left (negative) or right (positive)
     * relative to the driver.
     */
    public double getLeftRight() {
        return leftRight;
    }

    /**
     * Returns the desired rotation counterclockwise (negative) or
     * clockwise (positive).
     */
    public double getRotation() {
        return rotation;
    }

    public Button hoodUpButton() {
        return hoodUpButton_;
    }

    public Button hoodDownButton() {
        return hoodDownButton_;
    }

    /**
     * Returns a {@link Button} that, when pressed, will read critical values
     * from the <code>Shooter</code> tab of the {@link Shuffleboard}
     * (specifically, the flywheel speed and hood angle.)  This permits manual
     * override for testing purposes.
     */
    public Button readShuffleboardButton() {
        return readShuffleboardButton_;
    }

    /**
    * Toggles the intake/uptake on and off, switching to the appropriate
    * state as necessary.
    * @return Returns a {@link Button} object to use for command bindings.
    */
    public Button intakeTestButton() {
        return intakeToggleTestButton_;
    }

    /**
     * Manual trigger the fireButton to release the cargo.
     */
    public Button fireButton() {
        return fireButton_;
    }

    public Button getTurntableLeftButton() {
        return turntableLeftButton_;
    }

    public Button getTurntableRightButton() {
        return turntableRightButton_;
    }

    public Button getIntakeReverseButton() {
        return intakeReverseButton_;
    }

    public Button getIntakeDeployToggleButton() {
        return intakeDeployToggleButton_;
    }

    /**
     * Holding down this button causes the hook to extends upwards.
     */
    public Button getClimbUpButton() {
        return climbUpButton_;
    }

    /**
     * Holding down this button causes the hook to retract downwards.
     */
    public Button getClimbDownButton() {
        return climbDownButton_;
    }

    /**
     * When this {@link Button} is held down, the human using the auxiliary
     * controller can override some of the robot's otherwise automatic
     * behavior, such as turret aiming and firing.
     */
    public Button getManualOverrideButton() {
        return manualOverrideButton_;
    }

    /**
     * When the Y Button is pressed on the drive controller, the gyro will reset.
     * It is important to note that the driver must orient the robot so the intake
     * is facing away from them in order for this callibration to be correct.
     */
    public Button getGyroResetButton() {
        return resetGyroButton_;
    }

    /**
     * Returns true if there is no human input on the controller -- in other
     * words, if {@link InputSubsystem#getLeftRight getLeftRight()},
     * {@link InputSubsystem#getFrontBack getFrontBack()}, and
     * {@link InputSubsystem#getRotation getRotation()} are all in the dead zone.
     */
    public boolean joysticksAtNeutral() {
        if (frontBack == 0 && leftRight == 0 && rotation == 0) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Continuously check the human input devices and update our private variables.
     */
    @Override
    public void periodic() {
        frontBack = 0;
        leftRight = 0;
        rotation = 0;

        if (controllers != null && controllers[DRIVE_CONTROLLER_INDEX] != null) {

            // Allow the drive controller to control the overall output of the
            // SwerveDriveSubsystem.
            if (controllers[DRIVE_CONTROLLER_INDEX].getName() == "Logitech Extreme 3D") {
                // The joystick mappings are entirely different -- twist the joystick to rotate.
                frontBack += controllers[DRIVE_CONTROLLER_INDEX].getRawAxis(2);
                leftRight += controllers[DRIVE_CONTROLLER_INDEX].getRawAxis(1);
                rotation += controllers[DRIVE_CONTROLLER_INDEX].getRawAxis(4); // This does not seem to want to work!
            } else {
                // Normal XBox controllers.
                frontBack += controllers[DRIVE_CONTROLLER_INDEX].getLeftY();
                leftRight += controllers[DRIVE_CONTROLLER_INDEX].getLeftX();
                rotation += controllers[DRIVE_CONTROLLER_INDEX].getRightX();
            }
        }

        frontBack = deadzoneScale(frontBack, Constants.JOYSTICK_DEAD_ZONE);
        leftRight = deadzoneScale(leftRight, Constants.JOYSTICK_DEAD_ZONE);
        rotation = deadzoneScale(rotation, Constants.JOYSTICK_DEAD_ZONE);

        super.periodic();
    }

    /**
     * Scales values from the three joystick inputs from the Dead Zone to start at
     * zero, the minimum value, and get to 1, the maximum value. Not starting at 0
     * from the start and after the Dead Zone, reacing "10%."
     * @param channel
     * @param deadzone
     * @return
     */
    private static double deadzoneScale(double channel, double deadzone) {
        if (Math.abs(channel) >= 1) {
            channel = Math.signum(channel);
        } else if (Math.abs(channel) < deadzone) {
            channel = 0;
        } else {
            // Uses linear interpolation to determine the channel value based on the
            // dead zone.
            // It starts the increase in value at the deadzone, versus at the "origin."
            channel = linterp(channel, deadzone, 1.0);
            channel = exponentialResponseCurve(channel, Constants.JOYSTICK_RESPONSE_CURVE_EXPONENT);
        }
        return channel;
    }

    /**
     * Performs linear interpolation of the given value. This is a helper function for
     * {@link #deadzoneScale(double, double) deadzoneScale()}.
     *
     * @param current The current value.  It does not have to fall within the range
     *                [min, max].
     * @param min     The minimum value -- where the parameter of interpolation should be 0.
     * @param max     The maximum value -- where the parameter of interpolation should be 1.
     * @return        The parameter of interpolation: 0 at min, 1 at max, and any value
     *                inbetween or beyond.  The parameter will always have the same sign
     *                as the original 'current' argument.
     */
    private static double linterp(double current, double min, double max) {
        return Math.signum(current) * ((Math.abs(current) - min) / (max - min));
    }

    /**
     * Makes a value in the range [-1.0, 1.0] scale according to the given exponent.
     * This is a helper function for
     * {@link #deadzoneScale(double, double) deadzoneScale()}.
     *
     * @param channel A value between -1 and 1.
     * @param exponent The exponent to scale by, with 1.0 representing linear scaling.  The
     *                 exponent should be positive.
     * @return abs(channel) raised to the exponent power, but with the same sign as the
     *         original channel argument.
     */
    private static double exponentialResponseCurve(double channel, double exponent) {
        channel = Math.pow(Math.abs(channel), exponent) * Math.signum(channel);
        return channel;
    }
}
