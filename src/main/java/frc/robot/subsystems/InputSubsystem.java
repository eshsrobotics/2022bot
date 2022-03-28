package frc.robot.subsystems;

import java.rmi.server.ExportException;
import java.util.HashMap;
import java.util.Arrays;

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
    private XboxController[] controllers =  null;
    private double frontBack = 0;
    private double leftRight = 0;
    private double rotation = 0;
    private boolean aButton = false;

    private final static int DRIVE_CONTROLLER_INDEX = 0;
    private final static int AUXILARY_CONTROLLER_INDEX = 1;

    /**
     * A button that triggers a manual release, to the shooter, of a ball
     * captured by the indexer.  Nominally the release is automatic upon finding
     * a vision solution.
     */
    private Button fireButton_ = null;

    private Button hoodUpButton_ = null;
    private Button hoodDownButton_ = null;
    private Button turntableLeftButton_ = null;
    private Button turntableRightButton_ = null;
    private Button intakeToggleTestButton_ = null;
    private Button intakeReverseButton_ = null;
    private Button intakeDeployToggleButton_ = null;
    private Button climbUpButton_ = null;
    private Button climbDownButton_ = null;
    private Button manualOverrideButton_ = null;

    /**
     * Initializes this object and determines which input methods are usable.
     */
    public InputSubsystem() {
        // controller = new XboxController(Constants.XBOX_CONTROLLER_PORT);
        assignControllers(controllers);

        // Right now, there's only one controller.  That could be a problem later
        // when we have two controllers hooked up (one for driving, one for gunnery.)
        fireButton_ = new Button(() -> {
            //return controller.getRightBumper();
            return controllers[AUXILARY_CONTROLLER_INDEX].getRightTriggerAxis() > 0;
        });
        hoodUpButton_ = new Button(() -> {
            int dpadAngle = controllers[AUXILARY_CONTROLLER_INDEX].getPOV();
            return (dpadAngle == 0);
        });
        hoodDownButton_ = new Button(()->{
            int dpadAngle = controllers[AUXILARY_CONTROLLER_INDEX].getPOV();
            return (dpadAngle > 135 && dpadAngle < 225);
        });
        turntableLeftButton_ = new Button(() -> {
            return (controllers[AUXILARY_CONTROLLER_INDEX].getLeftBumper());
        });
        turntableRightButton_ = new Button(() -> {
            return (controllers[AUXILARY_CONTROLLER_INDEX].getRightBumper());
        });
        intakeToggleTestButton_ = new Button(() -> {
            return controllers[DRIVE_CONTROLLER_INDEX].getBButton();
        });
        intakeReverseButton_ = new Button (() -> {
            return controllers[DRIVE_CONTROLLER_INDEX].getXButton();
        });
        intakeDeployToggleButton_ = new Button (() -> {
            return controllers[DRIVE_CONTROLLER_INDEX].getAButton();
        });
        climbDownButton_ = new Button (() -> {
            return controllers[AUXILARY_CONTROLLER_INDEX].getBButton();
        });
        climbUpButton_ = new Button (() -> {
            return controllers[AUXILARY_CONTROLLER_INDEX].getAButton();
        });
        manualOverrideButton_ = new Button (() -> {
            return controllers[AUXILARY_CONTROLLER_INDEX].getLeftTriggerAxis() > 0;
        });

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("InputSubsystem");
        shuffleboardTab.addNumber("frontBack", () -> frontBack);
        shuffleboardTab.addNumber("leftRight", () -> leftRight);
        shuffleboardTab.addNumber("rotation", () -> rotation);
    }

    /**
     * Given this function to fill and array list for the priority of controllers. There are two
     * controllers, a drive controller and an auxiallary controller. The first contoller in this list 
     * is the drive controller, which is programmed to complete drive related functions. The second 
     * controller is the auxiallary controller, which is programmed to complete additional tasks. If only
     * one conroller is plugged in, it will be assigned the drive controller tasks, for it is more important
     * in competition than the auxilary functions. 
     * 
     * @param controllers This funtion will check to see what controller is assigned. This array list has
     *                    a total of two controllers that are determined by their name.  
     * 
     */
    private void assignControllers(XboxController[] controllers) {
        // There are 6 virtal ports in the drivers station that we could connect a controller to. 
        final int NUM_VIRTUAL_PORTS = 6;

        // Construct a dictionary that maps names to the index where we prefer
        // a controller of that name to be.  That is to say, our keys are name strings,
        // and our values are indices within controllers[].
        var table = new HashMap<String, Integer>();
        for (String s : Constants.DRIVE_CONTROLLER_NAME_PRIORITY) {                    
            table.put(s, 0); // Controllers[0] is the driver controller.
        }
        for (String s : Constants.AUXILARY_CONTROLLER_NAME_PRIORITY) {                    
            table.put(s, 1); // Controllers[1] is the auxiliary controller.
        }

        for (int i = 0; i < NUM_VIRTUAL_PORTS; ++i) {
            try {
                XboxController candidateController = new XboxController(i); 

                if (!candidateController.isConnected()) {
                    System.err.printf("Warning: Xbox controller disconnected virtual port %d", i);
                    continue;
                }

                // Look over all of the priority names, for both drive and auxiliary controllers,
                // to see if the current XBox controller matches one of them.  If so, assign as
                // appropriate.
                table.forEach((name, index) -> {
                    boolean driveControllerAssigned = (controllers[0] == null);
                    boolean auxiliaryControllerAssigned = (controllers[1] == null);

                    if (candidateController.getName() == name) {

                        if (!driveControllerAssigned && !auxiliaryControllerAssigned) {

                            // Nothing's assigned yet, so copy freely.
                            controllers[index] = candidateController;

                        } else if (!driveControllerAssigned && auxiliaryControllerAssigned) {

                            if (index == 0) {
                                controllers[0] = candidateController;
                            } else {
                                // There's already an auxiliary controller assigned.  Do nothing.
                            }

                        } else if (driveControllerAssigned && !auxiliaryControllerAssigned) {

                            if (index == 0) {
                                // The drive controller is already assigned.  Do nothing.
                            } else {
                                controllers[1] = candidateController;
                            }

                        } else if (driveControllerAssigned && auxiliaryControllerAssigned) {

                            // Both controllers are assigned.  Ignore everything else!
                        }
                    } else {
                        // No controller that has been plugged in was recognized by name. We are in this
                        // else loop to determine what the controller assignment will be.
                        if (!driveControllerAssigned) {
                            controllers[0] = candidateController;
                        } else if (!auxiliaryControllerAssigned) {
                            controllers[1] = candidateController;
                        } else {
                            // Both controllers are assigned. Ignore the controller input.
                        }
                    }                      
                 });                
                
            } catch (Exception e) {
                // An XBox controller was not found on virtual port i.
                // There is nothing further to do, we are going to skip the black
                // virtual ports and move on. 
            }
        } // end (for each virtual port)        

        System.out.printf("Summary:\n  Drive Controller : %s\n  Auxiliary Controller : %s", 
            controllers[0].getName(), 
            controllers[1].getName());
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

    public Button getClimbUpBotton() {
        return climbDownButton_;
    }

    public Button getClimbDownButton() {
        return climbUpButton_;
    }

    public Button getManualOverrideButton() {
        return manualOverrideButton_;
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
            frontBack += controllers[DRIVE_CONTROLLER_INDEX].getLeftY();
            leftRight += controllers[DRIVE_CONTROLLER_INDEX].getLeftX();
            rotation += controllers[DRIVE_CONTROLLER_INDEX].getRightX();
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
