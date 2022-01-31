package frc.robot.drivers;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.driveSchemes.DrivingScheme;

/**
 * SwerveDrivers take four
 * {@link SwerveModuleState Swerve Module States} and commands the robot to
 * drive with them.
 *
 * @see {@link DrivingScheme}, the interface which produces the
 * {@code SwerveModuleStates}
 */
public interface SwerveDriver {
    /**
     * Update the robot to the goal positions and speeds. This is meant to be
     * called during {@link SubsystemBase#periodic periodic()}, and may be called
     * multiple times per second.
     *
     * @param swerveModuleStates An array of four shopping cart angles and
     *                           speeds.
     * @see {@link frc.robot.Constants#FRONT_LEFT FRONT_LEFT}
     */
    public void drive(SwerveModuleState[] swerveModuleStates);
}