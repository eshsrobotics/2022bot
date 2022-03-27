package frc.robot.subsystems;

import java.util.Map;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    /**
     * The overall turning speed that we report to the outside world.
     */
    private double turnSpeed = 0;

    private double solutionDistance = 0;
    private boolean solutionFound = false;
    private NetworkTableEntry xEntry = null;
    private NetworkTableEntry yEntry = null;
    private NetworkTable table = null;

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

        table = NetworkTableInstance.getDefault().getTable("limelight");

        ShuffleboardTab shuffleboardTab = Shuffleboard.getTab("Vision");
        shuffleboardTab.addNumber("turnSpeed", () -> getTurnSpeed());
        shuffleboardTab.addBoolean("solutionFound", () -> solutionFound());
        shuffleboardTab.addNumber("distance (in.)", () -> getSolutionDistanceInches());
        xEntry = shuffleboardTab.add("x", 0).getEntry();
        yEntry = shuffleboardTab.add("y", 0).getEntry();
    }

    /**
     * Returns true if a vision solution was found and false otherwise.  Along
     * with {@link #getTurnSpeed()} and {@link #getSolutionDistanceInches()},
     * this function forms the main output of the vision subsystem.
     */
    public boolean solutionFound() {
        return solutionFound;
    }

    /**
     * Returns the speed by which you need to rotate in order to align the
     * camera with the vision target; values range between -1.0 and 1.0.
     * Along with {@link #solutionFound()} and
     * {@link #getSolutionDistanceInches()}, this function forms the main output
     * of the vision subsystem.
     */
    public double getTurnSpeed() {
        if (!solutionFound) {
            // It's meaningless to turn if we can't see a vision solution in the first place.
            return 0;
        } else {
            return -turnSpeed;
        }
    }

    /**
     * Returns the distance, in inches, from the center of the Limelight's
     * camera to the center of the vision solution.    Along
     * with {@link #solutionFound()} and {@link #getTurnSpeed()},
     * this function forms the main output of the vision subsystem.
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

        // Get the X and Y variables, too, just for debugging (they're not actually part of the vision solution)
        xEntry.setDouble(visionSolution.get("x"));
        yEntry.setDouble(visionSolution.get("y"));

        solutionDistance = visionSolution.get("dist");
        solutionFound = visionSolution.get("solutionFound") == 0 ? false : true;
        if (solutionFound) {
            turnSpeed = useSolution(visionSolution);
        }
    }

    //////////////////////
    // Private methods. //
    //////////////////////

    /**
     * Returns this subsystem's official vision solution.  This allows us to
     * change from the {@link #getSolutionFromLimelight() "production solution"}
     * to the {@link #getSolutionFromGyro(double) "debug solution"} with one
     * line of code.
     */
    private Map<String, Double> getSolution() {
        return getSolutionFromLimelight();
        // return getSolutionFromGyro(0);
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
        // if (pidController.atSetpoint()) {
        //     return 0;
        // }

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

        double ty = table.getEntry("ty").getDouble(0.0);
        double tx = table.getEntry("tx").getDouble(0.0);
        double tv = table.getEntry("tv").getDouble(0.0);

        double dist = (Constants.VISION_TARGET_AVERAGE_HEIGHT_INCHES -
                            Constants.LIMELIGHT_LENS_HEIGHT_INCHES) /
                            Math.sin((Constants.LIMELIGHT_MOUNT_ANGLE_DEGREES + ty) *
                                     Constants.DEGREES_TO_RADIANS);

        return Map.of("x",              Double.valueOf(tx),
                      "y",              Double.valueOf(ty),
                      "solutionFound",  Double.valueOf(tv),
                      "dist",           Double.valueOf(dist));
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


            // double gyroAngleDegrees = gyro.getAngle();
            // return Map.of("x",             gyroAngleDegrees - pretendAngleDegrees,
            //               "y",             Double.valueOf(0),
            //               "solutionFound", Double.valueOf(1.0),
            //               "dist",          Double.valueOf(100));

            // Sometimes, apropos of nothing, we get "HAL: Incompatible State: The operation cannot be completed" messages
            // when we try to read from the gyro.  I don't know what causes these.  I don't care, either.  The robot shouldn't
            // blow up just because we can't get a freaking angle.
            //
            return Map.of("x",             Double.valueOf(0),
                          "y",             Double.valueOf(-1),
                          "solutionFound", Double.valueOf(0.0),
                          "dist",          Double.valueOf(666)); // The distance will help us realize that something is amiss.

    }
}
