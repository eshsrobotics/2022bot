package frc.robot;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
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

        // The "crab rotation" vectors are the four vectors that are tangent to
        // the circumcircle of the drive base rectangle.  If you'll forgive the
        // gratuitous ASCII:
        //
        //                  ,
        //                 /
        //                /
        //               /    ,---------.
        //              / ,--'           `--.
        //  45.00°     ',---------------------..     135.00°
        //            /:                       :\
        //           / |                       | \
        //          :  |                       |  \
        //          |  |                       |  |\
        //          |  |                       |  | `
        //          |  |                       |  |
        //        . |  |                       |  |
        //         \|  |                       |  |
        //          \  |                       |  ;
        //           \ |                       | /
        //            \:                       ;/
        // -45.00°     ``---------------------',     -135.00°
        //                `--.           ,--' /
        //                    `---------'    /
        //                                  /
        //                                 /
        //                                '
        //
        // Here, the drive base's dimensions are equal, so the tangents all lie at
        // 45-degree angles.  The formula for calculating these angles is simple and
        // actually does involve tangents.

        double theta = Math.atan(Constants.WHEEL_BASE_LENGTH_METERS / Constants.WHEEL_BASE_WIDTH_METERS);
        // double phi = Math.PI/2 - theta;               // This term cancels out.
        double[] thetas = { 0, 0, 0, 0 };
        thetas[Constants.FRONT_LEFT] = theta;
        thetas[Constants.FRONT_RIGHT] = Math.PI - theta; //  π/2 + φ = (π/2 - θ) + π/2 = π - θ
        thetas[Constants.BACK_RIGHT] = theta - Math.PI;  // -π/2 - φ = (θ - π/2) - π/2 = θ - π
        thetas[Constants.BACK_LEFT] = -theta;

        // Translate the crab rotation angles into vectors.
        Translation2d[] crabRotationVectors = { null, null, null, null };
        for (int i = 0; i < thetas.length; ++i) {
            // We assume here that the up vector faces forward.  The rotation
            // here is counterclockwise around the origin.
            double angleRadians = thetas[i] > 0 ? thetas[i] : Math.PI + thetas[i];
            crabRotationVectors[i] = new Translation2d(0, -1).rotateBy(new Rotation2d(angleRadians));
        }
        return null;
    }

}
