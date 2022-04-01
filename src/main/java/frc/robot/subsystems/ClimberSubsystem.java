package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * The ClimberSubsystem class is created to control and own the climber on
 * the robot. It is responsible for pulling the robot up and letting the robot
 * down with the climber mechanism.
 */
public class ClimberSubsystem extends SubsystemBase {

    private CANSparkMax climbMotor = null;

    public ClimberSubsystem() {
        // The robot must be in break mode (blue CAN light), or else the robot
        // will plummet and break.
        climbMotor = new CANSparkMax(Constants.CLIMBER_CAN_ID, MotorType.kBrushless);
    }

    public void climberUp() {
        climbMotor.set(Constants.CLIMBER_SPEED);
    }

    public void climberDown() {
        climbMotor.set(-Constants.CLIMBER_SPEED);
    }

    public void climberStop() {
        climbMotor.stopMotor();
    }

    public void periodic() {

    }


}