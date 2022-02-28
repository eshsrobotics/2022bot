package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * This subsystem is used for all the hooded shooter functions, including
 * the turret, hooded shooter angles, and flywheel speed.
 */
public class ShooterSubsystem extends SubsystemBase {
   /**
    * The amount by which {@link #currentHoodPosition} increments during each
    * iteration of {@link #periodic periodic()} -- this always has a value of
    * {@link #HOOD_INCREMENT_VALUE +HOOD_INCREMENT_VALUE},
    * {@link #HOOD_INCREMENT_VALUE -HOOD_INCREMENT_VALUE}, or 0.
    */
    private double currentHoodIncrement = 0;

    /**
     * The current position of the left and right servos that control the
     * hooded shooter's position, ranging between 0.0 and 1.0.
     */
    private double currentHoodPosition = 0;

    /**
     * This constant is used to control the speed at which the hooded shooter
     * raises and lowers.  When the user is not actively driving the position,
     * the hood will stop moving.
     */
    private static final double HOOD_INCREMENT_VALUE = 0.1;

    private PWM leftServo = null;
    private PWM rightServo = null;

    /**
     * Initializes the hooded shooter while simultaneously pulling down
     * the hooded shooter to its start position at zero.
     */
    public ShooterSubsystem() {
        // We are assuming that these two expressions never throw exceptions.
        leftServo = new PWM(Constants.HOOD_LEFT_SERVO_PORT);
        rightServo = new PWM(Constants.HOOD_RIGHT_SERVO_PORT);
    }

    @Override
    public void periodic() {

        // Make the hood raise or lower (or stand still) over time, until it
        // reaches the limit.
        currentHoodPosition += currentHoodIncrement;
        if (currentHoodPosition > 1) {
            currentHoodPosition = 1;
            currentHoodIncrement = 0;
        } else if (currentHoodPosition < 0) {
            currentHoodPosition = 0;
            currentHoodIncrement = 0;
        }
        leftServo.setPosition(currentHoodPosition);
        rightServo.setPosition(currentHoodPosition);
    }

    /**
     * Causes the hood to go up.
     */
    public void raiseHood() {
        currentHoodIncrement = HOOD_INCREMENT_VALUE;
    }

    /**
     * Causes the hood to go down.
     */
    public void lowerHood() {
        currentHoodIncrement = -HOOD_INCREMENT_VALUE;
    }

    /**
     * Causes the hood to stop.
     */
    public void stopHood() {
        currentHoodIncrement = 0;
    }
}
