package frc.robot;

import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Takes {@link InputSubsystem#getFrontBack frontBack}, leftRight, and rotation and converts them into our four
 * {@link SwerveModuleState swerve module states and speeds}.
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
