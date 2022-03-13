package frc.robot.subsystems;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
        }

        frontBack = deadzoneScale(frontBack, Constants.JOYSTICK_DEAD_ZONE);
        leftRight = deadzoneScale(leftRight, Constants.JOYSTICK_DEAD_ZONE);
        rotation = deadzoneScale(rotation, Constants.JOYSTICK_DEAD_ZONE);

        super.periodic();
    }

    /**
     * Initializes this object and determines which input methods are usable.
     */
    public InputSubsystem() {
        controller = new XboxController(Constants.XBOX_CONTROLLER_PORT);
        if (!controller.isConnected()) {
            System.err.println("Warning: Xbox controller disconnected");
        }
        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("InputSubsystem");
        shuffleboardTab.addNumber("frontBack", () -> frontBack);
        shuffleboardTab.addNumber("leftRight", () -> leftRight);
        shuffleboardTab.addNumber("rotation", () -> rotation);
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
            // channel = (channel * channel) * Math.signum(channel);
            channel = ((Math.abs(channel) - deadzone) / (1 - deadzone)) * Math.signum(channel);

            // Make an exponential curve response.
            channel = Math.pow(Math.abs(channel), 2) * Math.signum(channel);
        }
        return channel;
    }
}
