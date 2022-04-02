package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
     * The portion of the shooter turntable speed that is governed by human
     * input.
     */
    private double turntableManualSpeed = 0;

    /**
     * The portion of the shooter turntable speed that is governed by the
     * vision subsystem.
     */
    private double turntableAutomaticSpeed = 0;

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
    private PWMMotorController upperFlywheel = null;

    private RelativeEncoder rightFlywheelEncoder = null;
    private RelativeEncoder leftFlywheelEncoder = null;

    /**
     * Overall speed of the flywheels, between 0.0 and 1.0.
     */
    private double flyWheelSpeedRight = 0;
    private double flyWheelSpeedLeft = 0;

    private static final double P = 1;
    private static final double I = 0.01;
    private static final double D = 0.00;

    private NetworkTableEntry flywheelSpeedEntry = null;
    private NetworkTableEntry hoodAngleEntry = null;

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
        upperFlywheel = new Spark(Constants.SHOOTER_UPPER_FLYWHEEL_PWM_PORT);

        // Intentionally stop the flywheels.
        flyWheelSpeedLeft = 0;
        flyWheelSpeedRight = 0;
        accelerateFlywheels();

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
        shuffleboardTab.addNumber("turntableManualSpeed", () -> turntableManualSpeed);
        shuffleboardTab.addNumber("turntableAutomaticSpeed", () -> turntableAutomaticSpeed);
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
        hoodAngleEntry = shuffleboardTab.add("Set Hood Angle", Double.valueOf(0)).getEntry();
    }

    /**
     * Allows external systems to set the speed for the flywheel for both motors.
     */
    public void setFlyWheelSpeed(double speed) {
        flyWheelSpeedLeft = MathUtil.clamp(speed, -1.0, 1.0);
        flyWheelSpeedRight = -MathUtil.clamp(speed, -1.0, 1.0);
    }

    public void setHoodAngle(double angle) {
        currentHoodPosition = angle;
    }
    /**
     * Reads shuffleboard entries related to the shooter and updates the flywheel speeds
     * and hood angles accordingly.  This is used for rapid testing purposes.
     */
    public void readFromShuffleboard() {
        flyWheelSpeedRight = -MathUtil.clamp(flywheelSpeedEntry.getDouble(0), -1.0, 1.0);
        flyWheelSpeedLeft = MathUtil.clamp(flywheelSpeedEntry.getDouble(0), -1.0, 1.0);
        currentHoodPosition = hoodAngleEntry.getDouble(0);
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

        // For the upper flywheel to spin at the perfered speed, use the {@link Constant} along with a
        // similar bottom flywheel value.
        final double uppperFlywheel_ = Constants.UPPER_FLYWHEEL_MUTIPLIER * flyWheelSpeedRight;
        upperFlywheel.set(uppperFlywheel_);
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
     * Sets the part of the shooter turntable's speed that is governed by human
     * input.
     *
     * It is important that we distinguish this from the turntable speed
     * governed by the vision subsystem; the manual speed is allowed to ignore
     * the limit switch, but the automatic speed is not.
     *
     * @param speed The new speed of the turntable.  If this is nonzero, we will
     *              ignore the {@link #setVisionTurnSpeed(double) vision
     *              turntable speed}.
     */
    public void setManualTurnSpeed(double speed) {
        turntableManualSpeed = MathUtil.clamp(speed, -1.0, 1.0);
    }

    /**
     * Sets the portion of the shooter turntable's speed which is governed by
     * the automatic adjustments of the vision subsystem.
     *
     * Unlike the {@link #setManualTurnSpeed(double) manual speed}, this speed
     * is subject to directional constraints from the
     * {@link #turntableLimitSwitch}.
     */
    public void setVisionTurnSpeed(double speed) {
        turntableAutomaticSpeed = MathUtil.clamp(speed, -1.0, 1.0);

        if (turntableAutomaticSpeed != 0 && !turntableLimitSwitch.get()) {
            // Remember the direction that the turntable was turning in last
            // while it was free to move.
            lastNonzeroTurntableSpeed = Math.signum(turntableAutomaticSpeed);
        }
    }

    /**
     * Helper function for {@link #periodic()}. Rotates the turntable in the
     * user's desired direction...if and only if we are permitted to.
     */
    private void maybeRotateTurntable() {
        double finalTurntableSpeed = 0;

        if (turntableManualSpeed != 0) {

            // Manually adjusting the turntable overrides vision control *and*
            // ignores the limit switch.
            finalTurntableSpeed = turntableManualSpeed;

        } else {

            finalTurntableSpeed = turntableAutomaticSpeed;

            if (turntableLimitSwitch.get()) {
                // Limit switch is hit; ban further motion in that direction of the
                // travel.
                double turntablePermittedDirection = -Math.signum(lastNonzeroTurntableSpeed);
                if ((turntablePermittedDirection < 0 && turntableAutomaticSpeed > 0) ||
                    (turntablePermittedDirection > 0 && turntableAutomaticSpeed < 0)) {
                    // Disallow (automatic) rotation of the turntable in illegal
                    // directions.
                    finalTurntableSpeed = 0;
                }
            }
        }

        if (Math.abs(finalTurntableSpeed) < EPSILON) {
            turntableMotor.stopMotor();
        } else {
            turntableMotor.set(finalTurntableSpeed);
        }
    }

    // private void CaculateHoodAngle() {
    //     double distance = Math.
    //
    //  }
    //
    // private void CaculateFlywheelSpeed() {
    //     double distance = 0;
    // }
}
