package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * A driving scheme takes three holonomic drive control channels
 * ({@link frc.robot.subsystems.InputSubsystem#getFrontBack() frontBack},
 * {@link frc.robot.subsystems.InputSubsystem#getLeftRight() leftRight}, and
 * {@link frc.robot.subsystems.InputSubsystem#getRotation() rotation}) and converts them into four
 * {@link SwerveModuleState swerve module states and speeds}, one for each swerve module.
 *
 * We can think of a couple of good ways to do this, so we abstracted the common features into
 * an interface.
 *
 * @see {@link frc.robot.subsystems.InputSubsystem InputSubsystem}, the
 * subsystem which produces the drive control channel values that we need
 * @see {@link frc.robot.SwerveDriver SwerveDriver}, the interface that
 * consumes the {@code SwerveModuleStates} which we generate
 */
public interface DrivingScheme {
    /**
     * Performs the conversion.
     *
     * @param frontBack The desired movement toward or away from the driver.
     * @param leftRight The desired movement left or right relative to the driver.
     * @param rotation  The desired clockwise or counterclockwise rotation of the robot.
     * @return An array of four turning and translational velocities.
     */
    public SwerveModuleState[] convert(double frontBack, double leftRight, double rotation);
}
