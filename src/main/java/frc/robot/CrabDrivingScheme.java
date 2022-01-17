package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * An alternative {@link frc.robot.DrivingScheme DrivingScheme} based on vector
 * addition rather than
 * {@link frc.robot.WPILibDrivingScheme WPILib built-in primitives}.  The idea
 * is that we add the overall robot translation vector to a multiple of the
 * crab rotation vectors in order to come up with overall direction vectors for
 * each swerve module.  The larger the rotation component of the input, the
 * larger the contribution of the crab rotation.
 */
public class CrabDrivingScheme implements DrivingScheme {

    @Override
    public SwerveModuleState[] convert(double frontBack, double leftRight, double rotation) {
        Translation2d absoluteTranslationVector = new Translation2d(leftRight, frontBack);

        // Define the direction vectors for all four swerve wheels. Initially,
        // they start as copies of the overall direction vector.
        Translation2d[] swerveTranslationVectors = new Translation2d[] {
            absoluteTranslationVector,
            absoluteTranslationVector,
            absoluteTranslationVector,
            absoluteTranslationVector
        };
        return null;
    }

}
