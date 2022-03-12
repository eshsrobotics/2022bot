package frc.robot.subsystems;

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
    private double frontBack = 0;
    private double leftRight = 0;
    private double rotation = 0;
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
            frontBack += controller.getLeftY();
            leftRight += controller.getLeftX();
            rotation += controller.getRightX();
            aButton = controller.getAButton();
        }

        // Clamp values that are too high.
        if (Math.abs(frontBack) > 1.0) {
            frontBack = Math.signum(frontBack);
        }
        if (Math.abs(leftRight) > 1.0) {
            leftRight = Math.signum(leftRight);
        }
        if (Math.abs(rotation) > 1.0) {
            rotation = Math.signum(rotation);
        }

        // Deadzone values that are too low
        if (Math.abs(frontBack) < Constants.JOYSTICK_DEAD_ZONE) {
            frontBack = 0;
        }
        if (Math.abs(leftRight) < Constants.JOYSTICK_DEAD_ZONE) {
            leftRight = 0;
        }
        if (Math.abs(rotation) < Constants.JOYSTICK_DEAD_ZONE) {
            rotation = 0;
        }
        super.periodic();
    }

    /**
     * Initializes this object and determines which input methods are usable.
     */
    public InputSubsystem() {
        controller = new XboxController(Constants.XBOX_CONTROLLER_PORT);
        if (!controller.isConnected()) {
            System.err.println("Warning: Xbox controller disconnected");
        } else {
            // Right now, there's only one controller.  That could be a problem later
            // when we have two controllers hooked up (one for driving, one for gunnery.)
            fireButton = new Button(() -> {
                return controller.getRightBumper();
            });
            hoodUpButton_ = new Button(() -> {
                int dpadAngle = controller.getPOV();
                return (dpadAngle == 0);
            });
            hoodDownButton_ = new Button(()->{
                int dpadAngle = controller.getPOV();
                return (dpadAngle > 135 && dpadAngle < 225);
            });
        }

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("InputSubsystem");
        shuffleboardTab.addNumber("frontBack", () -> frontBack);
        shuffleboardTab.addNumber("leftRight", () -> leftRight);
        shuffleboardTab.addNumber("rotation", () -> rotation);
        shuffleboardTab.addBoolean("aButton", () -> aButton);
    }
}
