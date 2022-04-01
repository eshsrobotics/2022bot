package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;

public class IntakeSubsystem extends SubsystemBase {
    private static final double NORMAL_INTAKING_SPEED = -1.0;
    private static final double REJECTION_INTAKING_SPEED = 1.0;
    private double intakeSpeed = NORMAL_INTAKING_SPEED;

    private enum StateValues {
        START,
        DEPLOY,
        INTAKE_UPTAKE_ON,
        INTAKE_UPTAKE_ON_BALL_IN_INDEXER,
        INTAKE_UPTAKE_ON_FIRING,
        INTAKE_UPTAKE_OFF,
        INTAKE_UPTAKE_OFF_BALL_IN_INDEXER,
        INTAKE_UPTAKE_ON_BALL_IN_INDEXER_AND_UPTAKE,
        INTAKE_UPTAKE_OFF_FIRING
    }
    private StateValues currentState = StateValues.START;

    private boolean receivedFireCommand = false;

    /**
     * True if the intake and uptake are active(pull balls in).
     */
    private boolean intakeAndUptakeEnabled = false;

    private static final double PNEUMATICS_DEPLOY_WAIT_TIME_SEC = 1.0;

    /**
     * Records when we entered the {@link StateValues#DEPLOY DEPLOY} state last;
     * this way, we assume after {@link #PNEUMATICS_DEPLOY_WAIT_TIME_SEC} amount of time the intake should
     * be deployed.
     *
     * A value of 0 is treated specially here: it indicates that we are entering
     * the DEPLOY state anew.  Otherwise, its value will be when we entered the DEPLOY
     * state last.
     */
    private double pneumaticsDeployStartTimeSec = 0;

    /**
     * The time passed, in seconds, after the indexer motors start turning.
     */
    private double indexerStartTimeSec = 0;

    /**
     * Okay, the indexer motors are on.  The ball goes <b>fwoomp!</b> and releases
     * to the shooter flywheel.  Nice!  Now, when do you turn those bad boys off?
     *
     * <ul>
     *   <li>
     *     Real answer: When the sensor says there's no ball there anymore.
     *   </li>
     *   <li>
     *     Temporary answer (since we don't have a sensor): Just wait for
     *     {@link #WAIT_INDEXER_SPIN_TIME_SEC} seconds and then assume that the
     *     ball is well and truly launched.
     *   </li>
     * </ul>
     */
    private static final double WAIT_INDEXER_SPIN_TIME_SEC = 3;

    private double redColorValue;

    private double greenColorValue;

    private double blueColorValue;

    /**
     * Color sensor will be used to detect which type of ball is entering our intake system.
     */
    private ColorSensorV3 colorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    /**
     * Color matcher will take the inputted values and use a 3d plane to estimate their distance from our
     * preset values of what the colors should be to determine if they are correct.
     */
    private final ColorMatch colorMatcher = new ColorMatch();

    /**
     * Preset values that will be used to check if the color our sensor is seeing is close enough to what we want it to be.
     */
    private final Color kBlueBall = new Color(0, 0, 0);
    private final Color kRedBall = new Color(0, 0, 0);


    /**
     * Allows us to swap whether we want to deploy and turn on our intake at the beginning of teleop or not.
     * Possible values are: {@link IntakeSubsystem#StateValues.INTAKE_UPTAKE_ON INTAKE_UPTAKE_ON},
     * {@link IntakeSubsystem#StateValues.INTAKE_UPTAKE_OFF INTAKE_UPTAKE_OFF}
     */
    private static final StateValues STATE_AFTER_DEPLOY = StateValues.INTAKE_UPTAKE_OFF;

    /**
     * The uptake motor controls the diagonal belt which brings balls from the ground level up to the parallel indexer belts.
     */
    private MotorController uptakeMotor = null;

    /**
     * The intake motor controls an axle with Mecanum rollers on either end.  Balls that the robot drives into are squished between
     * the intake rollers and the ground, driving them inexorably into the entrance to the uptake.
     */
    private MotorController intakeMotor = null;

    /**
     * This represents an adafruit IR breakbeam sensor (SKU number 2168) which will detect when a ball is in the indexer.
     * https://www.adafruit.com/product/2168
     */
    private DigitalInput indexerSensor = null;

    /**
     * Evidently the uptake sensor is of the same type as the indexer sensor.
     */
    private DigitalInput uptakeSensor = null;

    /**
     * left and right Indexer motor initializations
     */
    private CANSparkMax leftIndexerMotor = null;
    private CANSparkMax rightIndexerMotor = null;

    private Compressor comp = null;
    private DoubleSolenoid solenoidRight = null;
    private DoubleSolenoid solenoidLeft = null;

    /**
     * The state of the intake. If true, the intake is deployed. When we turn
     * the robot on, we expect the intake to be in the upright position (ie, false).
     */
    private boolean intakeDeployedState = false;

    public IntakeSubsystem() {
        colorMatcher.addColorMatch(kBlueBall);
        colorMatcher.addColorMatch(kRedBall);

        uptakeMotor = new PWMSparkMax(Constants.UPTAKE_MOTOR_PWM_PORT);
        intakeMotor = new PWMSparkMax(Constants.INTAKE_MOTOR_PWM_PORT);
        indexerSensor = new DigitalInput(Constants.INDEX_SENSOR_DIO_PORT);
        uptakeSensor = new DigitalInput(Constants.UPTAKE_SENSOR_DIO_PORT);

        var shuffleboardTab = Shuffleboard.getTab("Intake");
        shuffleboardTab.addBoolean("intakeUptakeEnabled", () -> intakeAndUptakeEnabled);
        shuffleboardTab.addBoolean("receivedFireCommand", () -> receivedFireCommand);
        shuffleboardTab.addNumber("pneumaticsDeployStartTimeSec", () -> pneumaticsDeployStartTimeSec);
        shuffleboardTab.addString("currentState", () -> currentState.toString());

        // For the Color Sensor.
        shuffleboardTab.addNumber("red", () -> colorSensor.getRed());
        shuffleboardTab.addNumber("green", () -> colorSensor.getGreen());
        shuffleboardTab.addNumber("blue", () -> colorSensor.getBlue());
        shuffleboardTab.addNumber("IR", () -> colorSensor.getIR());
        shuffleboardTab.addBoolean("BallInIndexer", () -> !indexerSensor.get());
        shuffleboardTab.addBoolean("BallInUptake", () -> !uptakeSensor.get());

        //indexer motors
        leftIndexerMotor = new CANSparkMax(Constants.INDEXER_ROLLER_LEFT_CAN_ID, MotorType.kBrushless);
        rightIndexerMotor = new CANSparkMax(Constants.INDEXER_ROLLER_RIGHT_CAN_ID, MotorType.kBrushless);

        // For the pneumatic solenoids.
        comp = new Compressor(1, PneumaticsModuleType.CTREPCM);
        solenoidRight = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
        solenoidLeft = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

        // Activate the compressor immediately. There are pressure gauges on the robot
        // that will automatically stop and start the compressor when the pressure dips
        // below the working and storage pressure.
        comp.enableAnalog(Constants.MINIMUM_COMPRESSOR_PSI, Constants.MAXIMUM_COMPRESSOR_PSI);
    }

    /**
     * Allows the use of a state machine that will call functions based on what the intake
     * function will be. These states can be to intake balls, reject balls, or be at
     * rest, waiting for the ball to be detected.
     */
    @Override
    public void periodic() {

        // Our intakes can be operating in forward or reverse mode, depending
        // on whether we are ingesting an enemy ball.
        intakeSpeed = NORMAL_INTAKING_SPEED;
        updateIntakeSpeed();

        switch(currentState) {
            case START:
                currentState = StateValues.DEPLOY;
                break;
            case DEPLOY:
                if (this.pneumaticsDeployStartTimeSec == 0) {
                    // We have just entered the DEPLOY state.
                    intakeMotor.stopMotor();
                    uptakeMotor.stopMotor();

                    if (IntakeSubsystem.STATE_AFTER_DEPLOY == StateValues.INTAKE_UPTAKE_OFF) {
                        // if we don't want to deploy our intake, just immediately go to intake uptake off state.
                        this.currentState = StateValues.INTAKE_UPTAKE_OFF;
                        pneumaticsDeployStartTimeSec = 0;
                    } else {
                        intakeAndUptakeEnabled = true;

                        // TODO: Pneumatics should be enabled at the start of autonomous,
                        // not immediately on robotInit() as this code does now.
                        // Effectively, the transition from START to DEPLOY needs
                        // to wait for a new external signal.
                        System.out.printf("Pneumatics Enabled\n");

                        // Set the start time if we just entered deploy
                        this.pneumaticsDeployStartTimeSec = Timer.getFPGATimestamp();
                    }
                }
                if (Timer.getFPGATimestamp() - pneumaticsDeployStartTimeSec >= PNEUMATICS_DEPLOY_WAIT_TIME_SEC) {
                    // checking if wait time for pneumatics has elapsed.
                    currentState = StateValues.INTAKE_UPTAKE_ON;
                    pneumaticsDeployStartTimeSec = 0;
                }
                break;
            case INTAKE_UPTAKE_ON:
                // As long as we're in this state, the intake and uptake should be moving.
                intakeMotor.set(intakeSpeed);
                uptakeMotor.set(1.0);

                if (!intakeAndUptakeEnabled) {
                    // Checking if intake and uptake was told to stop, then switching the state to off.
                    currentState = StateValues.INTAKE_UPTAKE_OFF;

                } else if (ballInIndexer() && intakeUptakeMotorFULL()) {
                    currentState = StateValues.INTAKE_UPTAKE_ON_BALL_IN_INDEXER;
                    this.receivedFireCommand = false;
                }
                break;

            case INTAKE_UPTAKE_ON_BALL_IN_INDEXER:
                if (!intakeAndUptakeEnabled) {
                    // Checking if intake and uptake was told to stop, then switching the state to off.
                    currentState = StateValues.INTAKE_UPTAKE_OFF;
                } else if (ballInIndexer() && ballInUptake()) {
                    // Switches states once there are the maximum amount of balls in the bot.
                    currentState = StateValues.INTAKE_UPTAKE_ON_BALL_IN_INDEXER_AND_UPTAKE;
                } else if (commandedToFire()) {
                    // If told to shoot, move to the firing state
                    currentState = StateValues.INTAKE_UPTAKE_ON_FIRING;
                }
                break;

            case INTAKE_UPTAKE_ON_FIRING:
                if (ballInIndexer()) {
                    System.out.print("Releasing Ball - spinning indexer forward\n");
                    leftIndexerMotor.set(Constants.INDEXER_SPEED);
                    rightIndexerMotor.set(-Constants.INDEXER_SPEED);
                } else {
                    leftIndexerMotor.stopMotor();
                    rightIndexerMotor.stopMotor();
                    currentState = StateValues.INTAKE_UPTAKE_ON;
                }
                break;

            case INTAKE_UPTAKE_OFF:
                // As long as we're in this state, the intake and uptake should not be moving.
                intakeMotor.stopMotor();
                uptakeMotor.stopMotor();

                if (intakeAndUptakeEnabled) {
                    // Checking if intake and uptake was told to stop, then switching the state to off.
                    currentState = StateValues.INTAKE_UPTAKE_ON;

                } else if (ballInIndexer() && intakeUptakeMotorFULL()) {
                    currentState = StateValues.INTAKE_UPTAKE_OFF_BALL_IN_INDEXER;
                    receivedFireCommand = false;
                }
                break;

            case INTAKE_UPTAKE_OFF_BALL_IN_INDEXER:
                if (intakeAndUptakeEnabled) {
                    // Checking if intake and uptake was told to stop, then switching the state to off.
                    currentState = StateValues.INTAKE_UPTAKE_ON;
                } else if (commandedToFire()) {
                    // If told to shoot, move to the firing state
                    currentState = StateValues.INTAKE_UPTAKE_OFF_FIRING;
                }
                break;

            case INTAKE_UPTAKE_ON_BALL_IN_INDEXER_AND_UPTAKE:
                // If control makes it here, we already had a ball in the indexer *and*
                // we detected another in the uptake.  That's two balls in our
                // digestive system; Rapid React rules forbid a robot from eating
                // more than two.  So we must politely decline any future chance of
                // of a meal.  For now.
                //intakeMotor.stopMotor();
                //uptakeMotor.stopMotor();

                if (commandedToFire()) {
                    currentState = StateValues.INTAKE_UPTAKE_ON_FIRING;
                    // Ball should be loaded
                } else if (!intakeAndUptakeEnabled) {
                    // Hitting the B Button while we are in the holding state while the
                    // robot is full, the robot is already stop. Transistion us from
                    // the holding state to the intak uptake off.
                    currentState = StateValues.INTAKE_UPTAKE_OFF;
                }
                break;

            case INTAKE_UPTAKE_OFF_FIRING:
                if (ballInIndexer()) {
                    System.out.print("Releasing Ball - spinning indexer forward\n");
                    leftIndexerMotor.set(Constants.INDEXER_SPEED);
                    rightIndexerMotor.set(-Constants.INDEXER_SPEED);
                } else {
                    System.out.print("Turning indexer off\n");
                    leftIndexerMotor.stopMotor();
                    rightIndexerMotor.stopMotor();
                    currentState = StateValues.INTAKE_UPTAKE_OFF;
                }
                break;
            } // periodic ends
        }

    /**
     * Changes intake wheel direction depeding on the color of the ball.
     * Uses color alliance and ball color from color sensor.
     */
    private void updateIntakeSpeed() {

        boolean blueBallDetected = false;
        boolean onBlueAlliance = (DriverStation.getAlliance() == DriverStation.Alliance.Blue);

        if (onBlueAlliance != blueBallDetected) {
            intakeSpeed = REJECTION_INTAKING_SPEED;
        }
        else {
            intakeSpeed = NORMAL_INTAKING_SPEED;
        }
    }

    /**
     * Toggles the intake and uptake motors.
     */
    public void toggleIntakeUptake() {
        if (this.intakeAndUptakeEnabled) {
            intakeAndUptakeEnabled = false;
        } else {
            intakeAndUptakeEnabled = true;
        }
    }

    /**
     * Detects if there is a ball in the indexer, using indexer sensor.
     */
    private boolean ballInIndexer() {
        return !indexerSensor.get();
    }

    /**
     * Tells if the intake/uptake are at full speed.
     */
    public boolean intakeUptakeMotorFULL() {
        // TODO: look at motor values or wait (to check if the intake motors are at full speed).
        // Going to fake it because we dont have either ^ .
        return true;
    }

    /**
     * Tells if the uptake is full.
     * @return Temporarily true all the time
     */
    public boolean ballInUptake() {
        // No sensor in the uptake. Will return false as we are unsure as to whether there is a ball
        return false;
    }

    /**
     * Tells the InputSubsystem to release the cargo into the hooded shooter's
     * fly wheels.
     */
    public void releaseCargoToShooter() {
        receivedFireCommand = true;
    }

    /**
     * Tells us if {@link #releaseCargoToShooter()} recently called.
     */
    public boolean commandedToFire() {
        return receivedFireCommand;
    }

    /**
     * Deploy or retract the intake assembly.
     * @param deploy True if the intake should be deployed out, and false if
     *               we should retract it inward.
     */
    public void deployIntake(boolean deploy) {
        if (deploy) {
            solenoidLeft.set(DoubleSolenoid.Value.kForward);
            solenoidRight.set(DoubleSolenoid.Value.kForward);
        } else {
            solenoidLeft.set(DoubleSolenoid.Value.kReverse);
            solenoidRight.set(DoubleSolenoid.Value.kReverse);
        }
        intakeDeployedState = deploy;
    }

    public boolean isIntakeDeployed() {
        return intakeDeployedState;
    }
}


