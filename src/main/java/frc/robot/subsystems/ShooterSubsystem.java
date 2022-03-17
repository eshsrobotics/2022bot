package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
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
     * The shooter turntable's current speed.  This is either equal to the last speed
     * set by {@link #turn(double)} or 0.0 (if we hit the limit switch.)
     */
    private double turntableSpeed = 0;

    /**
     * What directions should the turntable be allowed to rotate?
     *
     * Negative: Rotate counterclockwise only (we reached our clockwise limit.)
     * 0:        We may rotate in any direction.
     * Positive: Rotate clockwise only (we reached our counterclockwise limit.)
     */
    private int turntablePermittedDirection = 0;

    /**
     * This constant is used to control the speed at which the hooded shooter
     * raises and lowers.  When the user is not actively driving the position,
     * the hood will stop moving.
     */
    private static final double HOOD_INCREMENT_VALUE = 0.01;

    private PWM leftServo = null;
    private PWM rightServo = null;
    private DigitalInput turntableLimitSwitch = null;
    private CANSparkMax turntableMotor = null;

    /**
     * Initializes the hooded shooter while simultaneously pulling down
     * the hooded shooter to its start position at zero.
     */
    public ShooterSubsystem() {
        // We are assuming that these two expressions never throw exceptions.
        leftServo = new PWM(Constants.HOOD_LEFT_SERVO_PWM_PORT);
        rightServo = new PWM(Constants.HOOD_RIGHT_SERVO_PWM_PORT);

        turntableLimitSwitch = new DigitalInput(Constants.SHOOTER_TURN_TABLE_LIMIT_SWITCH_DIO_PORT);
        turntableMotor = new CANSparkMax(Constants.SHOOTER_TURNTABLE_CAN_ID, MotorType.kBrushless);
    }

    @Override
    public void periodic() {
        maybeRotateTurntable();

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
        System.out.printf("CurrentPosition = %.1f.  Increment set to %.1f.", currentHoodPosition, currentHoodIncrement);
    }

    /**
     * Causes the hood to go down.
     */
    public void lowerHood() {
        currentHoodIncrement = -HOOD_INCREMENT_VALUE;
        System.out.printf("CurrentPosition = %.1f.  Increment set to %.1f.", currentHoodPosition, currentHoodIncrement);
    }

    /**
     * Causes the hood to stop.
     */
    public void stopHood() {
        currentHoodIncrement = 0;
        System.out.printf("CurrentPosition = %.1f.  Increment set to %.1f.", currentHoodPosition, currentHoodIncrement);
    }

    /**
     * Starts turning the shooter turntable at the given speed.
     *
     * @param speed How quickly to turn.  The range is -1.0 (full speed counterclockwise)
     *              to 0.0 (stopped) to 1.0 (full speed clockwise.)
     */
    public void turn(double speed) {
        turntableSpeed = speed;
    }

    /**
     * Helper function for {@link #periodic()}.  Rotates the turntable in the
     * user's desired direction...if and only if we are permitted to.
     */
    private void maybeRotateTurntable() {

        if (turntableLimitSwitch.get()) {
            // Limit switch is hit; ban further motion in that direction of
            // travel.
            turntablePermittedDirection = (int)-Math.signum(turntableSpeed);
        } else {
            // As long as the limit switch is not hit, permit travel in all directions.
            turntablePermittedDirection = 0;
        }

        if ((turntablePermittedDirection < 0 && turntableSpeed > 0) ||
            (turntablePermittedDirection > 0 && turntableSpeed < 0)) {
            // Disallow rotation of the turntable in illegal directions.
            turntableSpeed = 0;
        }

        if (Math.abs(turntableSpeed) < 0.01) { // TODO: replace with an epsilon constant
            turntableMotor.stopMotor();
        } else {
            turntableMotor.set(turntableSpeed);
        }
    }
}
