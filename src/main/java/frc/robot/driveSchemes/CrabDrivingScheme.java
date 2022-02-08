package frc.robot.driveSchemes;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import frc.robot.Constants;

/**
 * An alternative {@link frc.robot.driveSchemes.DrivingScheme DrivingScheme} based on vector
 * addition rather than
 * {@link frc.robot.driveSchemes.WPILibDrivingScheme WPILib built-in primitives}.  The idea
 * is that we add the overall robot translation vector to a multiple of the
 * crab rotation vectors in order to come up with overall direction vectors for
 * each swerve module.  The larger the rotation component of the input, the
 * larger the contribution of the crab rotation.
 */
public class CrabDrivingScheme implements DrivingScheme {

    private Gyro gyro = null;

    /**
     * Initializes this object.
     *
     * @param gyro A {@link Gyro} object which is initialized such that the
     * front of the robot points away from the drivers when the gyro angle is 0.
     * Being consistent with this is very important for getting field-oriented
     * swerve working -- if the robot does not face forward when the gyro is
     * initialized, it will drive incorrectly for the remainder of the match.
     */
    CrabDrivingScheme(Gyro gyro) {
        this.gyro = gyro;
    }

    @Override
    public SwerveModuleState[] convert(double frontBack, double leftRight, double rotation) {

        // Clamp the input values to the range [-1, 1].
        frontBack = Math.min(1.0, Math.max(-1.0, frontBack));
        leftRight = Math.min(1.0, Math.max(-1.0, leftRight));
        rotation = Math.min(1.0, Math.max(-1.0, rotation));

        // This is where the driver wants to go with respect to the field's
        // reference frame.
        Translation2d absoluteTranslationVector = new Translation2d(leftRight, frontBack);

        // This is where the robot should be going with respect to ITS OWN
        // reference frame.  This is why we need the gyro.
        //
        // The WPILib Gyro class returns larger angles as the robot rotates
        // clockwise with respect to the top of the chassis.  Since we require
        // the robot to alway starts the match facing away from the driver's
        // side of the field, rotating the chassis by -gyro.getAngle() will
        // always face it in its starting direction.
        //
        // Rotate the joystick vector by the same amount.
        Translation2d adjustedTranslationVector = absoluteTranslationVector.rotateBy(gyro.getRotation2d());

        // Define the direction vectors for all four swerve wheels. Initially,
        // they start as copies of the overall direction vector.
        Translation2d[] swerveTranslationVectors = new Translation2d[] {
            adjustedTranslationVector,
            adjustedTranslationVector,
            adjustedTranslationVector,
            adjustedTranslationVector
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
        Translation2d[] finalVectors = { null ,null, null, null };
        final double r = Math.abs(rotation);
        for (int i = 0; i < thetas.length; ++i) {

            // Negative angles should rotate counterclockwise.
            double angleRadians = rotation > 0 ? thetas[i] : Math.PI + thetas[i];

            // We assume here that up vectors [Translate2d(0, -1)] point forward
            // with respect to the robot chassis.  The rotateBy() calls here
            // rotate counterclockwise around the origin.
            crabRotationVectors[i] = new Translation2d(0, -1).rotateBy(new Rotation2d(angleRadians));

            // Use the rotation channel to combine the translation and rotation
            // vectors.
            finalVectors[i] = adjustedTranslationVector.times(1 - r).plus(crabRotationVectors[i].times(r));
        }

        // Convert the vectors into SwerveModuleStates.
        return null;
    }

}
