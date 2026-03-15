package org.firstinspires.ftc.teamcode.subsystems.limelight;

/**
 * InertiaAutoAim computes the yaw correction needed to hit a stationary goal
 * when the robot itself is moving.
 *
 * <p>Problem: the robot is driving while it fires.  A ball fired straight ahead
 * will be displaced sideways by the robot's lateral velocity during the ball's
 * flight time.  To compensate, the aimer should lead the target by the amount
 * the ball will drift.
 *
 * <p>Algorithm (simplified):
 * <ol>
 *   <li>The robot (treated as the coordinate origin) is moving with velocity
 *       {@code robotVel} (3-D, in robot-frame coordinates).</li>
 *   <li>The goal is at position {@code goalPos} in the robot frame, computed
 *       from the given range and heading.</li>
 *   <li>During the ball's flight time {@code t = goalDistance / ballSpeed}, the
 *       robot (and therefore the "effective launch origin") moves by
 *       {@code robotVel * t}.</li>
 *   <li>The required aim point is the goal position <em>minus</em> that drift;
 *       {@link Math#atan2} on the resulting vector gives the corrected yaw.</li>
 * </ol>
 *
 * <p><b>Note:</b> This class is currently not used in the main codebase
 * (the reference in {@link Aimer} is commented out) but is available for
 * future inertia-compensation work.
 */
public class InertiaAutoAim {

    // -----------------------------------------------------------------------
    // Constants
    // -----------------------------------------------------------------------

    /**
     * Robot position in the local reference frame – always (0, 0, 0) because
     * all calculations are robot-relative.  The layout is {x, y, z} where y
     * is the vertical axis.
     */
    private final static double[] ROBOT_POS = {0, 0, 0};
    // double[] --> {x, y, z} (y is vertical)

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Creates an InertiaAutoAim instance.  No parameters are needed because
     * all state is passed per-call via {@link #getYawDegrees}.
     */
    public InertiaAutoAim() {
        // No member state to initialise; all inputs are provided per-call.
    }

    // -----------------------------------------------------------------------
    // Correction computation
    // -----------------------------------------------------------------------

    /**
     * Computes the yaw angle correction (in degrees) needed to compensate for
     * the robot's translational velocity while firing.
     *
     * <p>Coordinate convention: the robot is at (0, 0, 0).  X points to the
     * robot's right, Y points upward, and Z points forward (in the direction
     * the robot is heading).
     *
     * @param robotVel      robot velocity vector as {@code {vx, vy, vz}} in
     *                      inches per second (robot frame; y is vertical)
     * @param ballSpeed     muzzle speed of the ball in inches per second
     * @param robotYawRad   current robot heading in radians (field frame);
     *                      used to rotate the goal position into robot frame
     * @param goalDistance  straight-line distance from robot to goal in inches
     * @param goalElevation height of the goal above the robot in inches (vertical component)
     * @return yaw correction in degrees; positive = need to aim further left
     *         (counter-clockwise) to compensate for rightward robot motion
     */
    public double getYawDegrees(double[] robotVel, double ballSpeed, double robotYawRad, double goalDistance, double goalElevation) {
        // Compute the horizontal (ground-plane) distance from robot to goal,
        // stripping the vertical elevation component out of the total slant range.
        double baseDistance = Math.sqrt(goalDistance * goalDistance - goalElevation * goalElevation);

        // Represent the goal's position in the robot's local 3-D frame.
        // X = right of robot = baseDistance * sin(heading) (lateral field component)
        // Y = elevation = goalElevation (vertical)
        // Z = forward of robot = baseDistance * cos(heading) (forward field component)
        double[] goalPos = {
            baseDistance * Math.sin(robotYawRad),  // horizontal offset to the side
            goalElevation,                          // vertical offset to the goal
            baseDistance * Math.cos(robotYawRad)   // horizontal offset forward
        };
        // goalPos is relative to the robot position (which is ROBOT_POS = {0,0,0})

        // Estimate ball flight time: assume constant speed and no air resistance.
        double time = (goalDistance / ballSpeed); // seconds for the ball to reach the goal

        // Compute where the aim point must be to hit the goal given that the robot
        // (launch origin) will have drifted by robotVel * time during the ball's flight.
        // dx and dz are the lead-compensated horizontal aim deltas.
        double dx = goalPos[0] - (ROBOT_POS[0] + robotVel[0] * time); // X: right of robot
        double dz = goalPos[2] - (ROBOT_POS[2] + robotVel[2] * time); // Z: forward of robot

        // atan2 gives the compensated heading needed to reach the corrected aim point.
        // Subtracting robotYawRad converts it from field heading to a yaw correction delta.
        double yaw = Math.atan2(dz, dx) - robotYawRad;

        return Math.toDegrees(yaw); // convert radians to degrees for the caller
    }
}