package frc.robot.subsystems;

import java.security.spec.EncodedKeySpec;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.Constants;

/**
 * This subsystem is used for all the hooded shooter functions, including
 * the turret, hooded shooter angles, and flywheel speed.
 */
public class ShooterSubsystem extends SubsystemBase {

    private static final double EPSILON = 0.01;

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
     * This variable records the last speed the turntable had when it was actually turning.  This is what we compare in order to determine
     * the direction in which to forbid movement when the limit switch is pressed.
     */
    private double lastNonzeroTurntableSpeed = 0;

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
    private CANSparkMax leftFlyWheel = null;
    private CANSparkMax rightFlyWheel = null;
    private SparkMaxPIDController rightFlywheel_pidController = null;
    private SparkMaxPIDController leftFlywheel_pidController = null;

    private RelativeEncoder rightFlywheelEncoder = null;
    private RelativeEncoder leftFlywheelEncoder = null;

    /**
     * Overall speed of the flywheels, between 0.0 and 1.0.
     */
    private double flyWheelSpeedRight = 0;
    private double flyWheelSpeedLeft = 0;

    private static final double P = 0.01;
    private static final double I = 0.01;
    private static final double D = 0.00;

    private NetworkTableEntry flywheelSpeedEntry = null;

    private enum turntableStates {
        FREE_TO_MOVE_STATE,
        RESTRICT_MOVE_UNTIL_RELEASED
    }

    private turntableStates currentState = turntableStates.FREE_TO_MOVE_STATE;

    /**
     * Initializes the hooded shooter while simultaneously pulling down
     * the hooded shooter to it"s start position at zero.
     */
    public ShooterSubsystem() {
        // We are assuming that these two expressions never throw exceptions.
        leftServo = new PWM(Constants.HOOD_LEFT_SERVO_PWM_PORT);
        rightServo = new PWM(Constants.HOOD_RIGHT_SERVO_PWM_PORT);

        // Initialize the shooter flywheels.  We have two -- one on each side.
        leftFlyWheel = new CANSparkMax(Constants.FLYWHEEL_RIGHT_CAN_ID, MotorType.kBrushless);
        rightFlyWheel = new CANSparkMax(Constants.FLYWHEEL_LEFT_CAN_ID, MotorType.kBrushless);

        // Initializing the PIDcontroller to the respective motor CAN Spark Max.
        rightFlywheel_pidController = rightFlyWheel.getPIDController();
        leftFlywheel_pidController = leftFlyWheel.getPIDController();

        leftFlywheelEncoder = leftFlyWheel.getEncoder();
        rightFlywheelEncoder = rightFlyWheel.getEncoder();

        leftFlywheel_pidController.setP(P);
        leftFlywheel_pidController.setI(I);
        leftFlywheel_pidController.setD(D);

        rightFlywheel_pidController.setP(P);
        rightFlywheel_pidController.setI(I);
        rightFlywheel_pidController.setD(D);

        turntableLimitSwitch = new DigitalInput(Constants.SHOOTER_TURN_TABLE_LIMIT_SWITCH_DIO_PORT);
        turntableMotor = new CANSparkMax(Constants.SHOOTER_TURNTABLE_CAN_ID, MotorType.kBrushless);

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Shooter");
        shuffleboardTab.addNumber("turntableSpeed", () -> turntableSpeed);
        shuffleboardTab.addNumber("lastNonzeroTurntableSpeed", () -> lastNonzeroTurntableSpeed);
        shuffleboardTab.addNumber("turntablePermittedDirection", () -> turntablePermittedDirection);
        shuffleboardTab.addNumber("currentHoodPosition", () -> currentHoodPosition);
        shuffleboardTab.addNumber("currentHoodIncrement", () -> currentHoodIncrement);
        shuffleboardTab.addBoolean("turntableLimitSwitch", () -> turntableLimitSwitch.get());
        shuffleboardTab.addNumber("leftFlyWheelVelocity", () -> leftFlywheelEncoder.getVelocity());
        shuffleboardTab.addNumber("rightFlyWheelVelocity", () -> rightFlywheelEncoder.getVelocity());
        shuffleboardTab.addNumber("leftFlywheelSpeed", () -> flyWheelSpeedLeft);
        shuffleboardTab.addNumber("rightFlywheelSpeed", () -> flyWheelSpeedRight);
        flywheelSpeedEntry = shuffleboardTab.add("Set Flywheel Velocity", Double.valueOf(0)).getEntry();

    }

    /**
     * Allows external systems to set the speed for the flywheel for both motors.
     */
    public void setFlyWheelSpeed(double speed) {
        flyWheelSpeedLeft = speed;
        flyWheelSpeedRight = -speed;
    }

    /**
     * Gets the shuffleboard number and overrides the flywheel motors to go that
     * fast- for testing purposes.
     * @return
     */
    public void readFromShuffleboard() {
        flyWheelSpeedRight = -0.444; // -flywheelSpeedEntry.getDouble(0);
        flyWheelSpeedLeft = flywheelSpeedEntry.getDouble(0);

    }

    @Override
    public void periodic() {
        maybeRotateTurntable();
        accelerateFlywheels();
        adjustHood();
    }

    private void adjustHood() {
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
     * Helper function for {@link #periodic()}.  Instantaneously adjust the
     * speeds of the left and right (bottom) flywheel motors so they match a
     * target velocity, regardless of battery power or other constraints.
     */
    private void accelerateFlywheels() {
        // leftFlywheel_pidController.setReference(flyWheelSpeedLeft, CANSparkMax.ControlType.kVelocity);
        // rightFlywheel_pidController.setReference(flyWheelSpeedRight, CANSparkMax.ControlType.kVelocity);
        leftFlyWheel.set(flyWheelSpeedLeft);
        rightFlyWheel.set(flyWheelSpeedRight);
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
        if (Math.abs(turntableSpeed) > EPSILON) {
            lastNonzeroTurntableSpeed = turntableSpeed;
        }
    }

    /**
     * Helper function for {@link #periodic()}. Rotates the turntable in the
     * user's desired direction...if and only if we are permitted to.
     */
    private void maybeRotateTurntable() {
        switch (currentState) {
            case FREE_TO_MOVE_STATE:
                // If the limit switch is pressed when the robot is not moving, meaning
                // that the robot was turned on when the switch was pressed, there is no
                // way to know what direction to move based on previous action.
                // We are contemplating throwing an exeption and forcing the robot to quit
                // because it is dangerous to guess which direction the robot can turn
                // without messing up, damaging, or harming robot hardware.
                if (turntableLimitSwitch.get()) {
                    // Limit witch is hit; ban furter motion in that direction of the
                    // travel.
                    turntablePermittedDirection = (int) -Math.signum(turntableSpeed);
                    currentState = turntableStates.RESTRICT_MOVE_UNTIL_RELEASED;
                }
                break;
            case RESTRICT_MOVE_UNTIL_RELEASED:
                if (!turntableLimitSwitch.get()) {
                    // When the limit switch is not being pressed, meaning it has been
                    // released, the turntable moves back to the FREE_TO_MOVE_STATE.
                    currentState = turntableStates.FREE_TO_MOVE_STATE;
                } else {
                    if ((turntablePermittedDirection < 0 && turntableSpeed > 0) ||
                            (turntablePermittedDirection > 0 && turntableSpeed < 0)) {
                        // Disallow rotation of the turntable in illegal directions.
                        turntableSpeed = 0;
                    }
                }
                break;
        }
        if (Math.abs(turntableSpeed) < EPSILON) {
            turntableMotor.stopMotor();
        } else {
            turntableMotor.set(turntableSpeed);
        }

    }
}
