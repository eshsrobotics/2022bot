package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase{

    private enum StateValues {
        START,
        DEPLOY,
        INTAKE_UPTAKE_ON,
        INTAKE_UPTAKE_ON_BALL_IN_INDEXER,
        INTAKE_UPTAKE_ON_FIRING,
        INTAKE_UPTAKE_OFF,
        INTAKE_UPTAKE_OFF_BALL_IN_INDEXER,
        INTAKE_UPTAKE_OFF_FIRING
    }
    private StateValues currentState = StateValues.START;
    private boolean intakeAndUptakeEnabled = false;
    private static final double PNEUMATICS_DEPLOY_WAIT_TIME_SEC = 1.0;
    private double pneumaticsDeployStartTimeSec = 0;
    private double fakeIndexerDelayStartTime = 0;
    private static final double FAKE_INDEXER_DELAY_SEC = 1.0;


    /**
     * Allows us to swap whether we want to deploy and turn on our intake at the beginning of teleop or not.
     * Possible values are: {@link IntakeSubsystem#StateValues.INTAKE_UPTAKE_ON INTAKE_UPTAKE_ON},
     * {@link IntakeSubsystem#StateValues.INTAKE_UPTAKE_OFF INTAKE_UPTAKE_OFF}
     */
    private static final StateValues STATE_AFTER_DEPLOY = StateValues.INTAKE_UPTAKE_ON;



    public IntakeSubsystem() {

    }
    @Override
    public void periodic() {

        switch(currentState) {
            case START:
                currentState = StateValues.DEPLOY;
                break;
            case DEPLOY:
                if (this.pneumaticsDeployStartTimeSec == 0) {
                    // We have just entered the DEPLOY state.
                    if (IntakeSubsystem.STATE_AFTER_DEPLOY == StateValues.INTAKE_UPTAKE_OFF) {
                        // if we don't want to deploy our intake, just immediatley go to intake uptake off state.
                        this.currentState = StateValues.INTAKE_UPTAKE_OFF;
                        pneumaticsDeployStartTimeSec = 0;
                    } else {
                        intakeAndUptakeEnabled = true;
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
                if (!intakeAndUptakeEnabled) {
                    // Checking if intake and uptake was told to stop, then switching the state to off.
                    currentState = StateValues.INTAKE_UPTAKE_OFF;

                } else if (ballInIndexer() && intakeUptakeMotorFULL()) {
                    currentState = StateValues.INTAKE_UPTAKE_ON_BALL_IN_INDEXER;
                }
                break;

            case INTAKE_UPTAKE_ON_BALL_IN_INDEXER:
                if (!intakeAndUptakeEnabled) {
                    // Checking if intake and uptake was told to stop, then switching the state to off.
                    currentState = StateValues.INTAKE_UPTAKE_OFF;
                }
                if (commandedToFire()) {
                    // If told to shoot, move to the firing state
                    currentState = StateValues.INTAKE_UPTAKE_ON_FIRING;
                }

                break;

            case INTAKE_UPTAKE_ON_FIRING:
                if (this.fakeIndexerDelayStartTime == 0) {
                    // We just entered the UPTAKE state.
                    fakeIndexerDelayStartTime = Timer.getFPGATimestamp();
                    System.out.print("Releasing Ball - spinning indexer forward\n");
                }
                if (Timer.getFPGATimestamp() - fakeIndexerDelayStartTime >= FAKE_INDEXER_DELAY_SEC) {
                    // If control is here, ball has been shot
                    //
                    // TODO: Replace with a test that actually uses the indexer sensor.
                    currentState = StateValues.INTAKE_UPTAKE_ON;
                    System.out.print("Turning indexer on\n");
                }
                break;


            case INTAKE_UPTAKE_OFF:
                if (intakeAndUptakeEnabled) {
                    // Checking if intake and uptake was told to stop, then switching the state to off.
                    currentState = StateValues.INTAKE_UPTAKE_ON;

                } else if (ballInIndexer() && intakeUptakeMotorFULL()) {
                    currentState = StateValues.INTAKE_UPTAKE_OFF_BALL_IN_INDEXER;
                }
                break;

            case INTAKE_UPTAKE_OFF_BALL_IN_INDEXER:
                if (intakeAndUptakeEnabled) {
                    // Checking if intake and uptake was told to stop, then switching the state to off.
                    currentState = StateValues.INTAKE_UPTAKE_ON;
                }
                if (commandedToFire()) {
                    // If told to shoot, move to the firing state
                    currentState = StateValues.INTAKE_UPTAKE_OFF_FIRING;
                }

                break;

            case INTAKE_UPTAKE_OFF_FIRING:
                if (this.fakeIndexerDelayStartTime == 0) {
                    // We just entered the UPTAKE state.
                    fakeIndexerDelayStartTime = Timer.getFPGATimestamp();
                    System.out.print("Releasing Ball - spinning indexer forward\n");
                }
                if (Timer.getFPGATimestamp() - fakeIndexerDelayStartTime >= FAKE_INDEXER_DELAY_SEC) {
                    // If control is here, ball has been shot
                    //
                    // TODO: Replace with a test that actually uses the indexer sensor.
                    currentState = StateValues.INTAKE_UPTAKE_OFF;
                    System.out.print("Turning indexer on\n");
                }
                break;
            }
        }


    /**
     * Detects if there is a ball in the indexer.
     */
    private boolean ballInIndexer() {
        // TODO: When indexer sensor is ready, implement method properly.
        // Not what real ballInIndexer will return, using this to call the method.
        return true;
    }

    /**
     * Tells if the intake/uptake are at full speed.
     */
    public boolean intakeUptakeMotorFULL() {
        // TODO: look at motor values or wait (to check if the intake motors are at full speed).
        // Going to fake it because we dont have either ^ .
        return true;
    }

    public boolean commandedToFire() {
        // TODO: check if a different subsystem instructed us to fire.
        // Going to fake it because we dont have either ^ .
        return true;
    }


}
