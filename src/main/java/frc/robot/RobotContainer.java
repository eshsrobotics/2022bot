// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.InputSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private InputSubsystem inputSubsystem = null;
  private SwerveDriveSubsystem swerveDriveSubsystem = null;
  private ShooterSubsystem shooterSubsystem = null;
  private VisionSubsystem visionSubsystem = null;
  private Gyro gyro = null;
  private IntakeSubsystem intakeSubsystem = null;
  private ClimberSubsystem climberSubsystem = null;

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    gyro = new ADXRS450_Gyro();
    inputSubsystem = new InputSubsystem();
    this.swerveDriveSubsystem = new SwerveDriveSubsystem(inputSubsystem, gyro);
    shooterSubsystem = new ShooterSubsystem();
    visionSubsystem = new VisionSubsystem(gyro);
    intakeSubsystem = new IntakeSubsystem();
    climberSubsystem = new ClimberSubsystem();

    // Calibrate the gyro when the robot turns on.
    gyro.calibrate();

    // Configure the button bindings
    configureButtonBindings();

    // Instructs the shooter subsystem to rotate the turret based on input from the visionSubsystem (Can be overriden by manual input from second driver)
    shooterSubsystem.setDefaultCommand(new RunCommand(() -> {
      // Will re-enable, but testing manual control of the turntable with left and right triggers first.
      //
      double speedFromVision = visionSubsystem.getTurnSpeed() * 0.1;
      if (speedFromVision != 0) {
        shooterSubsystem.setVisionTurnSpeed(speedFromVision);
      }

      // TODO: Add code here to automatically adjust the hood and the flywheel speed
      // depending on what the vision solution's distance is.
    }, visionSubsystem, shooterSubsystem));
  }

  /**
   * The default position of the swerve modules is facing straight forward, with the gear side of the modules
   * pointing inward.  We call that value "0 degrees" for each swerve module.  This function causes the drive
   * to seek that state.
   */
  public void zeroPosition() {
    swerveDriveSubsystem.initialPosition();
    gyro.reset();
  }

  /** Temporary kludge. */
  public void reassignControllers() {
    inputSubsystem.assignControllersSimplistically();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    // Use the up and down buttons on the D-pad to manually control the hood,
    // for now.
    Button hoodUpButton = inputSubsystem.hoodUpButton();
    if (hoodUpButton != null) {
      hoodUpButton.whenHeld(new InstantCommand(() -> {
        shooterSubsystem.raiseHood();
      }));
      hoodUpButton.whenReleased(new InstantCommand(() -> {
        shooterSubsystem.stopHood();
      }));
    }

    Button hoodDownButton = inputSubsystem.hoodDownButton();
    if (hoodDownButton != null) {
      hoodDownButton.whenHeld(new InstantCommand(() -> {
        shooterSubsystem.lowerHood();
      }));
      hoodDownButton.whenReleased(new InstantCommand(() -> {
        shooterSubsystem.stopHood();
      }));
    }

    // Pressing the "Y" button on the auxiliary controller will read the
    // flywheel speed and hood angle from special variables on the "Shooter"
    // tab of the Shuffleboard.
    Button shuffleboardButton = inputSubsystem.readShuffleboardButton();
    if(shuffleboardButton != null) {
      shuffleboardButton.whenPressed(() -> {
        shooterSubsystem.readFromShuffleboard();
      });
    }


    // Use the BButton to turn the uptake/intake on and off.
    Button intakeTestButton = inputSubsystem.intakeTestButton();
    if (intakeTestButton != null) {
      intakeTestButton.whenPressed(() -> {
        intakeSubsystem.toggleIntakeUptake();
      });
    }

    // The A Button of the Drive Controller deploys and retracts the intake.
    Button intakeDeployToggleButton = inputSubsystem.getIntakeDeployToggleButton();
    if (intakeDeployToggleButton != null) {
      intakeDeployToggleButton.whenPressed(() -> {
        intakeSubsystem.deployIntake(!intakeSubsystem.isIntakeDeployed());
      });
    }

    /////////////////////////////////////////////////////////////////
    // Manual overrides.                                           //
    //                                                             //
    // These only work while the manual override button is active. //
    /////////////////////////////////////////////////////////////////

    if (inputSubsystem.getTurntableLeftButton() != null) {
      inputSubsystem.getTurntableLeftButton().whenHeld(new StartEndCommand(() -> {
        // Rotate the shooter turntable counterclockwise, but only if the
        // manual override is held down.
        Button manualOverrideButton = inputSubsystem.getManualOverrideButton();
        if (manualOverrideButton != null && manualOverrideButton.get()) {
          shooterSubsystem.setManualTurnSpeed(-0.25);
        }
      }, () -> {
        // Releasing will stop the turntable even without feedback from
        // the manual override. This adheres to the Principle of Least
        // Astonishment.
        shooterSubsystem.setManualTurnSpeed(0);
      }));
    }

    if (inputSubsystem.getTurntableRightButton() != null) {
      inputSubsystem.getTurntableRightButton().whenHeld(new StartEndCommand(() -> {
        // Rotate the shooter turntable clockwise, but only if the manual
        // override is held down.
        Button manualOverrideButton = inputSubsystem.getManualOverrideButton();
        if (manualOverrideButton != null && manualOverrideButton.get()) {
          shooterSubsystem.setManualTurnSpeed(+0.25);
        }
      }, () -> {
        // See above.
        shooterSubsystem.setManualTurnSpeed(0);
      }));
    }

    Button fireButton = inputSubsystem.fireButton();
    if (fireButton != null) {
      fireButton.whenPressed(() -> {
        // Pressing (and then releasing) the fire button will manually fire
        // the ball, but only when the manual override is held down at the
        // same time.
        Button manualOverrideButton = inputSubsystem.getManualOverrideButton();
        if (manualOverrideButton != null && manualOverrideButton.get()) {
          intakeSubsystem.releaseCargoToShooter();
        }
      });
    }

    // Holding down the climbDownButton retracts the climber, releasing it stops the climber.
    Button climbDownButton = inputSubsystem.getClimbDownButton();
    if (climbDownButton != null) {
      climbDownButton.whenHeld(new StartEndCommand(() -> {
        climberSubsystem.climberDown();
      }, () -> {
        climberSubsystem.climberStop();
      }));
    }

    // Holding down the climbUpButton extends the climber, releasing it stops the climber.
    Button climbUpButton = inputSubsystem.getClimbUpButton();
    if (climbUpButton != null) {
      climbUpButton.whenHeld(new StartEndCommand(() -> {
        climberSubsystem.climberUp();
      }, () -> {
        climberSubsystem.climberStop();
      }));
    }
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
