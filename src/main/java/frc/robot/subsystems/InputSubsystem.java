package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
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
    private XboxController controller = null;
    private Joystick joystick = null;
    private double frontBack = 0;
    private double leftRight = 0;
    private double rotation = 0;
    private DoubleSupplier frontBackChannel = null;
    private DoubleSupplier leftRightChannel = null;
    private DoubleSupplier rotationChannel = null;
    private boolean aButton = false;

    /**
     * A button that triggers a manual release, to the shooter, of a ball
     * captured by the indexer.  Nominally the release is automatic upon finding
     * a vision solution.
     */
    private Button fireButton = null;

    private Button hoodUpButton_ = null;
    private Button hoodDownButton_ = null;

    /**
     * Initializes this object and determines which input methods are usable.
     */
    public InputSubsystem() {
        detectController();
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("InputSubsystem");
        shuffleboardTab.addNumber("frontBack", () -> frontBack);
        shuffleboardTab.addNumber("leftRight", () -> leftRight);
        shuffleboardTab.addNumber("rotation", () -> rotation);
        shuffleboardTab.addBoolean("aButton", () -> aButton);
    }

    /**
     * Returns the desired movement toward (negative) or away (positive) from the driver.
     */
    public double getFrontBack() {
        return frontBack;
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

    public boolean getAButton() {
        return aButton;
    }

    public Button hoodUpButton() {
        return hoodUpButton_;
    }

    public Button hoodDownButton() {
        return hoodDownButton_;
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

        // If the XBox Controller is connected, allow its input to contribute to the overall
        // output.
        if (controller != null && controller.isConnected()) {
            frontBack += frontBackChannel.getAsDouble();
            leftRight += leftRightChannel.getAsDouble();
            rotation += rotationChannel.getAsDouble();
            aButton = controller.getAButton();
        }

        frontBack = deadzoneScale(frontBack, Constants.JOYSTICK_DEAD_ZONE);
        leftRight = deadzoneScale(leftRight, Constants.JOYSTICK_DEAD_ZONE);
        rotation = deadzoneScale(rotation, Constants.JOYSTICK_DEAD_ZONE);

        super.periodic();
    }

    /**
     * Detects whether an XBox controller or a joystick is connected to the system and,
     * most importantly, sets the lambda functions that {@link #periodic()} will
     * use to actually get the raw values.  <code>periodic()</code> never touches the
     * <code>jooystick</code> or <code>controller</code objects directly!
     */
    private void detectController() {
        XboxController controller = new XboxController(Constants.XBOX_CONTROLLER_PORT);
        boolean controllerDetected = false;

        if (!controller.isConnected()) {
            System.err.println("Warning: Xbox controller disconnected");
        } else {
            this.controller = controller;
            System.out.printf("Xbox controller name is: %s\n", controller.getName());
            for (int i = 0; i < Constants.KNOWN_CONTROLLER_NAMES.length; i++) {
                if (controller.getName() == Constants.KNOWN_CONTROLLER_NAMES[i]) {
                    controllerDetected = true;
                    break;
                }
            }
        }

        if (controllerDetected == true) {
            // We have found an XBox controller...or at least something that we have
            // tested as one.
            //
            // Right now, there's only one controller.  That could be a problem later
            // when we have two controllers hooked up (one for driving, one for gunnery.)

            // Note: Pushing the vertical channels of an XBox controller up causes it
            // to return a negative number (think of the Cartesian coordinate system.)
            // But our getFrontBack() documentation, which we built the rest of the code
            // around, says that positive values point away from the driver.
            // We flip the sign of frontBackChannel _only_ to compensate.
            frontBackChannel = () -> -this.controller.getLeftY();
            leftRightChannel = () -> this.controller.getLeftX();
            rotationChannel = () -> this.controller.getRightX();

            fireButton = new Button(() -> {
                return this.controller.getRightBumper();
            });
            hoodUpButton_ = new Button(() -> {
                int dpadAngle = this.controller.getPOV();
                return (dpadAngle == 0);
            });
            hoodDownButton_ = new Button(()->{
                int dpadAngle = this.controller.getPOV();
                return (dpadAngle > 135 && dpadAngle < 225);
            });
        } else {
            // Sadly, this is not a *known* XBbox controller, so we have no choice but
            // to assume that this thing is some sort of joystick.
            controller = null;
            Joystick joystick = new Joystick(Constants.XBOX_CONTROLLER_PORT);
            if (!joystick.isConnected()) {
                System.err.println("Warning: Joystick disconnected");

                // TODO: Handle getting values from NetworkTablesInput.
            } else {
                this.joystick = joystick;
                frontBackChannel = () -> -this.joystick.getY();
                leftRightChannel = () -> this.joystick.getX();
                rotationChannel = () -> this.joystick.getZ();

                fireButton = new Button(() -> this.joystick.getTrigger());
                hoodUpButton_ = new Button(() -> this.joystick.getThrottle() < 0);
                hoodDownButton_ = new Button(() -> this.joystick.getThrottle() > 0);
            }
        }
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
            // Uses linear interpalation to determine the channel value based on the
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
     * @param max     The maximum value -- where the parameter of inteprolation should be 1.
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
