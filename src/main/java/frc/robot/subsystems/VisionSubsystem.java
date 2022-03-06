package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This subsystem's purpose is to passively align the robot's shooting
 * turntable, as well as possible, to the vision target identified by the
 * Limelight camera.
 * <p>To do this, we use two levels of abstraction:</p>
 * <ul>
 *   <li>A "solution map", which can be obtained either
 *   {@link #getSolutionFromLimelight() from the Limelight} or, for debugging
 *   purposes, {@link #getSolutionFromGyro(double) the gyro}; and</li>
 *   <li>A PID controller that tells us how much we need to spin in order to
 *   make that solution map's <code>x</code> value be 0.</li>
 * </ul>
 * <p>This information in turn can be fed into some other method to perform the
 * actual rotation.<p>
 */
public class VisionSubsystem extends SubsystemBase {

    private Gyro gyro = null;
    private PIDController pidController = null;
    private double turnSpeed = 0;
    private double solutionDistance = 0;

    private static final double P = 1.0;
    private static final double I = 1.0;
    private static final double D = 0.01;

    /**
     * Initializes this object and its PID controller.
     * @param gyro A gyro to use for
     *             {@link #getSolutionFromGyro(double) getSolutionFromGyro()},
     *             which is actually only used for debugging.
     */
    public VisionSubsystem(Gyro gyro) {
        this.gyro = gyro;
        pidController = new PIDController(P, I, D);
    }

    /**
     * Returns the speed by which you need to rotate in order to align the
     * camera with the vision target.  Values range between -1.0 and 1.0.  This
     * is one of the two main outputs of this subsystem (the other being
     * {@link #getSolutionDistanceInches()}.)
     */
    public double getTurnSpeed() {
        return turnSpeed;
    }

    /**
     * Returns the distance, in inches, from the center of the Limelight's
     * camera to the center of the vision solution.  This is one of the two main
     * outputs of this subsystem (the other being {@link #getTurnSpeed()}).
     */
    public double getSolutionDistanceInches() {
        return solutionDistance;
    }

    /**
     * Continuously updates the vision subsystem based on whatever the camera
     * is telling us.  You can retrieve the results by calling
     * {@link #getTurnSpeed()} and {@link #getSolutionDistanceInches()}.
     */
    @Override
    public void periodic() {
        Map<String, Double> visionSolution = getSolution();
        solutionDistance = visionSolution.get("dist");
        turnSpeed = useSolution(visionSolution);
    }

    /**
     * Returns this subsystem's official vision solution.  This allows us to
     * change from the {@link #getSolutionFromLimelight() "production solution"}
     * to the {@link #getSolutionFromGyro(double) "debug solution"} with one
     * line of code.
     */
    private Map<String, Double> getSolution() {
        // return getSolutionFromLimelight();
        return getSolutionFromGyro(0);
    }

    /**
     * Using a
     * {@link edu.wpi.first.math.controller.PIDController PIDController},
     * calculate at what speed we should be rotating to align the camera with
     * the vision solution.  This function does not care whether it is the
     * turntable or the chassis that does the rotation.
     *
     * TODO: What is an idea tuning for the PID response?  Alignment within a second?
     *
     * @param visionSolution A {@link Map} containing the following keys:
     *                      <dl>
     *                        <dt><code>solutionFound</code></dt>
     *                        <dd>1.0 if a vision solution was found and 0.0
     *                        otherwise.  The values in the other keys are
     *                        meaningless if a vision solution does not
     *                        exist.</dd>
     *
     *                        <dt><code>x</code></dt>
     *                        <dd>The horizontal 'distance' from the center of the
     *                        camera to the center of the vision solution.  A
     *                        positive value means that the solution is to the
     *                        right of the camera center.</dd>
     *
     *                        <dt><code>y</code></dt>
     *                        <dd>The vertical 'distance' from the center of the
     *                        camera to the center of the vision solution.  A
     *                        positive value means that the solution is above
     *                        the camera center.</dd>
     *
     *                        <dt><code>dist</code></dt>
     *                        <dd>The distance to the vision solution in
     *                        inches.</dd>
     *                      </dl>
     * @return Returns the speed that the camera should be rotating, with values
     *         ranging between -1.0 and 1.0.  This is really a speed for some
     *         motor (or even an entire drive) doing the rotation.  Positive
     *         values should map to clockwise rotation of the chassis or
     *         turntable and negative values should map to counterclockwise
     *         rotation.
     */
    private double useSolution(Map<String, Double> visionSolution) {

        // Even though the block comment above made this function seem
        // complicated, it actually couldn't be simpler.  We simply use PID to
        // get us from whatever the solution measured as our X deviation to an
        // X deviation of 0.
        if (pidController.atSetpoint()) {
            return 0;
        }

        double delta = pidController.calculate(visionSolution.get("x"), 0);

        // Depending on what's in getSolution(), delta could have units of
        // degrees or (horizontal) pixels.  Whatever the case, our own output
        // will always be between -1 and 1.
        return MathUtil.clamp(delta, -1.0, 1.0);
    }

    /**
     * Returns a {@link #useSolution(Map) solution map} from the Limelight
     * vision camera.
     */
    private Map<String, Double> getSolutionFromLimelight() {

        // Technically speaking, both the setpoint and the measured value are in
        // units of (horizontal) pixels!
        pidController.setSetpoint(0);
        return null;
    }

    /**
     * Returns a {@link #useSolution(Map) solution map} using the gyro.  This is
     * a hack intended for debugging the PID control loop; using this function,
     * the robot will always point in the same direction in spite of any human
     * driver's attempt to do otherwise!
     *
     * @param pretendAngleDegrees We assume that when the gyro returns this
     *                            angle, we are aligned with the "vision
     *                            solution."  0, for instance, would use the
     *                            robot's orientation at power-on as the fixed
     *                            solution position.
     */
    private Map<String, Double> getSolutionFromGyro(double pretendAngleDegrees) {

        pidController.setSetpoint(pretendAngleDegrees);

        if (!pidController.isContinuousInputEnabled()) {
            // When using the gyro, our setpoint and measurements are both
            // angles in degrees.  Since angles wrap around, make sure the
            // PIDController knows this.
            //
            // Note that what we get out of the Limelight (as opposed to this
            // hack) is not an angle, but a distance in pixels.
            pidController.enableContinuousInput(-180, 180);
        }

        return Map.of("x",             gyro.getAngle() - pretendAngleDegrees,
                      "y",             Double.valueOf(0),
                      "solutionFound", Double.valueOf(1.0),
                      "dist",          Double.valueOf(100));
    }
}
