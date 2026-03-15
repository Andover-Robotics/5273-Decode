package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;


/**
 * Aimer computes the heading-correction power needed to point the robot at
 * a scoring goal using the Road Runner localizer instead of live camera data.
 *
 * <p>Field coordinate convention used by this class:
 * <ul>
 *   <li>Bottom-left (red human player zone) = (0, 0)</li>
 *   <li>Top-right (near red goal) = (144, 144)</li>
 *   <li>Bottom-right (blue human player zone) = (144, 0)</li>
 *   <li>Top-left (near blue goal) = (0, 144)</li>
 *   <li>All distances are in inches (field is 12 ft × 12 ft = 144 in × 144 in)</li>
 * </ul>
 *
 * <p>Workflow:
 * <ol>
 *   <li>Call {@link #setRedTarget()} or {@link #setBlueTarget()} to aim at
 *       the alliance-specific goal.</li>
 *   <li>Call {@link #relocalize()} when the robot is at the human player zone
 *       to re-zero the Road Runner localizer at a known field position.</li>
 *   <li>Each loop iteration, call {@link #calculateLocalizedTurnPower()} to
 *       get a {@code double[3]} array: {@code {turnPower, range, bearingDeg}}.</li>
 *   <li>Feed {@code turnPower} into {@link org.firstinspires.ftc.teamcode.subsystems.Movement}
 *       as the {@code turnCorrection} parameter.</li>
 * </ol>
 *
 * <p>A PIDF controller is used for the heading loop; all gains and the target
 * pose can be tuned live in FTC Dashboard because the class is annotated with
 * {@link Config}.
 */
@Config
public class Aimer {

    // -----------------------------------------------------------------------
    // PIDF gains and filter
    // -----------------------------------------------------------------------

    /** Proportional gain for the heading PID loop.
     *  Higher values produce faster corrections but risk oscillation. */
    public static double kP = 0.06;

    /** Integral gain for the heading PID loop.  Accumulates bearing error over
     *  time to eliminate steady-state offset.  Currently 0. */
    public static double kI = 0.0;

    /** Derivative gain for the heading PID loop.  Damps rapid bearing changes.
     *  Currently 0; a low-pass filter on the derivative is also applied. */
    public static double kD = 0.0;

    /** Feedforward gain: a small constant power applied in the direction of the
     *  bearing error to overcome static friction in the drive motors. */
    public static double kF = 0.0;

    /** Derivative smoothing coefficient (0–1).
     *  1.0 = no smoothing (raw derivative); 0.0 = maximum smoothing (holds last value).
     *  The exponential moving average is: {@code d = (1 - filter)*prevD + filter*rawD}. */
    public static double filter = 0.867;

    /** Maximum absolute integral value (anti-windup clamp).
     *  Prevents the integral from accumulating large values when the robot is far off-aim. */
    public static double maxIntegral = 1.0;

    /** Bearing deadband (degrees).  When |error| < deadband no correction is applied
     *  and the integral is reset to prevent chattering when the robot is aimed well. */
    public static double deadband = 1; // degrees

    // -----------------------------------------------------------------------
    // PID internal state
    // -----------------------------------------------------------------------

    /** Accumulated bearing error integral (degrees × seconds). */
    private double integral = 0;

    /** Smoothed derivative of the bearing error from the last call. */
    private double lastDerivative = 0.0;

    /** Bearing error on the previous iteration (degrees), used to compute the derivative. */
    private double lastError = 0;

    /** System time (ms) of the last {@link #calculateTurnPowerFromBearing(double)} call.
     *  Used to compute dt for the I and D terms. */
    private long lastTimestamp = 0;

    // -----------------------------------------------------------------------
    // Dependencies
    // -----------------------------------------------------------------------

    /** Road Runner mecanum drive; provides the localizer pose estimate. */
    private final MecanumDrive drive;

    // -----------------------------------------------------------------------
    // Target configuration
    // -----------------------------------------------------------------------

    /** Field-relative pose of the target AprilTag (scoring goal).
     *  Updated by {@link #setRedTarget()} or {@link #setBlueTarget()}.
     *  Default heading of 90° means the tag faces the field's +Y direction. */
    public static Pose2d tagPose = new Pose2d(0, 132, Math.toRadians(90));

    /** Height of the robot's camera/sensor above the floor (inches). */
    public static double cameraHeight = 11.815; // inches

    /** Height of the goal AprilTag above the floor (inches).
     *  Used to compute the 3-D range from the 2-D horizontal distance. */
    public static double goalAprilTagHeight = 29.5; // inches

    /** Horizontal distance (inches) from the back field wall to the aiming point.
     *  Adjusts where on the field the robot tries to aim "at" the goal from. */
    public static double goalBack = 4; // inches from back wall of the field

    /** Lateral distance (inches) from the side border (driver station wall) to
     *  the aiming point.  Positions the goal target away from the wall. */
    public static double goalOut = 15; // inches from driver side wall

    // -----------------------------------------------------------------------
    // Goal selection
    // -----------------------------------------------------------------------

    /**
     * Enum representing the two alliance goals on the field.
     */
    public enum Goal {
        /** Red alliance scoring goal (top-right area of the field). */
        RED,
        /** Blue alliance scoring goal (top-left area of the field). */
        BLUE
    }

    /** The currently selected alliance goal.  Written by {@link #setRedTarget()}
     *  and {@link #setBlueTarget()}; read by {@link #relocalize()}. */
    public static Goal selectedGoal = Goal.RED;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Creates an Aimer and immediately configures it for the red goal.
     * Also re-zeros the localizer pose to the robot's assumed starting position
     * at the human player zone.
     *
     * @param mecanumDrive the Road Runner drive used to read the localizer pose
     */
    public Aimer(MecanumDrive mecanumDrive) {
        this.drive = mecanumDrive;
        // Default to the red goal on construction; can be changed before a match.
        setRedTarget();
        // Re-zero the localizer at the human player starting position.
        relocalize();
    }

    // -----------------------------------------------------------------------
    // Goal selection helpers
    // -----------------------------------------------------------------------

    /**
     * Configures the aimer to target the <b>red alliance goal</b> (top-right
     * corner, approximately (144−goalOut, 144−goalBack) on the field).
     * Also switches to the red AprilTag pipeline implicitly via the caller.
     */
    public void setRedTarget(){
        selectedGoal = Goal.RED;
        // Compute the red goal's field position from the back/side offsets.
        tagPose = new Pose2d(144-goalOut, 144-goalBack, Math.toRadians(90));
    }

    /**
     * Configures the aimer to target the <b>blue alliance goal</b> (top-left
     * corner, approximately (goalOut, 144−goalBack) on the field).
     */
    public void setBlueTarget(){
        selectedGoal = Goal.BLUE;
        // Compute the blue goal's field position from the back/side offsets.
        tagPose = new Pose2d(goalOut, 144-goalBack, Math.toRadians(90));
    }

    /**
     * Returns the currently selected alliance goal.
     *
     * @return {@link Goal#RED} or {@link Goal#BLUE}
     */
    public Goal getGoal() {
        return selectedGoal;
    }

    // -----------------------------------------------------------------------
    // Localization
    // -----------------------------------------------------------------------

    /**
     * Re-zeros the Road Runner localizer at the robot's known starting position
     * in the human player zone.
     *
     * <p>The robot is assumed to be flush against the human player wall,
     * oriented at 90° (facing the field).  A {@code botThick} offset accounts
     * for the robot's physical half-width so that the localizer's (x,y) refers
     * to the robot center, not the field boundary.
     */
    public void relocalize(){
        // Half-width of the robot in inches; used to place the center away from the wall.
        int botThick = 9;
        if(selectedGoal == Goal.RED){
            // Red human player zone: bottom-left corner, heading 90° (facing +Y).
            drive.localizer.setPose(new Pose2d(0+botThick, 0+botThick, Math.toRadians(90)));
        } else if (selectedGoal == Goal.BLUE){
            // Blue human player zone: bottom-right corner, heading 90° (facing +Y).
            drive.localizer.setPose(new Pose2d(144-botThick, 0+botThick, Math.toRadians(90)));
        }
    }

    // -----------------------------------------------------------------------
    // Main aiming computation
    // -----------------------------------------------------------------------

    /**
     * Computes the turn correction power, range, and bearing to the goal using
     * the current localizer pose estimate.
     *
     * <p>Steps:
     * <ol>
     *   <li>Read the robot's current field pose from the Road Runner localizer.</li>
     *   <li>Compute the 2-D horizontal vector from robot to the goal tag pose.</li>
     *   <li>Combine with the camera–goal height difference to get the 3-D range.</li>
     *   <li>Compute the desired heading (atan2 of the goal vector).</li>
     *   <li>Subtract the robot's current heading to get the bearing error.</li>
     *   <li>Run the PID controller on the bearing to produce a turn power.</li>
     * </ol>
     *
     * @return {@code double[]{turnPower, range, bearing}} where:
     *         <ul>
     *           <li>{@code turnPower} – correction for the robot's yaw [-1, 1];
     *               positive = turn right (counterclockwise as seen from above)</li>
     *           <li>{@code range} – 3-D straight-line distance from robot to goal in inches</li>
     *           <li>{@code bearing} – angular error in degrees (positive = goal is to the right)</li>
     *         </ul>
     */
    public double[] calculateLocalizedTurnPower() {
        // Get the robot's current estimated field pose from the Road Runner localizer.
        Pose2d robotPose = drive.localizer.getPose();

        // 2-D vector from robot to goal tag (field coordinates, inches).
        double dx = tagPose.position.x - robotPose.position.x;
        double dy = tagPose.position.y - robotPose.position.y;

        // Horizontal (ground-plane) distance from robot to goal.
        double horizontalDistance = Math.hypot(dx, dy);

        // Height difference between the goal tag and the camera mounting height.
        double dz = goalAprilTagHeight - cameraHeight;

        // 3-D straight-line distance: combines horizontal and vertical separation.
        double range = Math.hypot(horizontalDistance, dz);

        // Desired robot heading to face the goal (radians, field frame).
        double desiredHeading = Math.atan2(dy, dx);

        // Current robot heading from the localizer (radians, field frame).
        double currentHeading = robotPose.heading.toDouble();

        // Bearing = angular difference between desired and current heading (degrees).
        double bearing = Math.toDegrees(desiredHeading - currentHeading);
        bearing = angleWrapDegrees(bearing); // wrap to [-180, 180]

        // Compute the PID turn power (negated so positive bearing → positive turn output).
        double turnPower = -calculateTurnPowerFromBearing(bearing);
        return new double[]{turnPower, range, bearing};
    }

    // -----------------------------------------------------------------------
    // Angle helpers
    // -----------------------------------------------------------------------

    /**
     * Wraps a degree angle into the range (-180, 180].
     *
     * <p>Used to ensure heading errors are always expressed as the shortest
     * angular path (avoiding ±360 jumps).
     *
     * @param angle raw angle in degrees
     * @return equivalent angle in (-180, 180]
     */
    private double angleWrapDegrees(double angle) {
        // Shift by 180, take mod 360, then shift back so result is in (-180, 180].
        return (angle + 180) % 360 - 180;
    }

    // -----------------------------------------------------------------------
    // PID controller
    // -----------------------------------------------------------------------

    /**
     * Runs the PIDF heading controller and returns a motor power in [-1, 1].
     *
     * <p>If the bearing is {@link Double#NaN} (e.g., tag not visible) all PID
     * state is reset and 0 is returned.  If the bearing is within the
     * {@link #deadband} the integral is reset and 0 is returned.
     *
     * @param bearing current heading error in degrees;
     *                {@link Double#NaN} if aiming data is unavailable
     * @return turn correction power in [-1.0, 1.0]; positive turns right
     */
    public double calculateTurnPowerFromBearing(double bearing) {
        // If bearing is NaN (e.g., tag lost), reset all PID state and return no correction.
        if (Double.isNaN(bearing)) {
            integral = 0;          // clear integral accumulation
            lastError = 0;         // clear previous error
            lastDerivative = 0;    // clear smoothed derivative
            lastTimestamp = 0;     // force dt to be treated as first call on resume
            return 0.0;            // no correction while tag is not visible
        }

        // Wrap bearing to (-180, 180] to ensure shortest-path error.
        double error = angleWrapDegrees(bearing);

        if (Math.abs(error) < deadband) {
            // Within the deadband: on-target enough; reset integral and return 0.
            integral = 0;
            lastError = error;      // keep last error so derivative is smooth on next entry
            lastDerivative = 0;     // reset derivative since we're stationary in the band
            return 0; // no correction needed
        }

        // ---- Compute dt ----
        long currentTime = System.currentTimeMillis();
        double deltaTime;
        if (lastTimestamp == 0) {
            deltaTime = 0.01; // assume 10 ms on the very first call (no previous timestamp)
        } else {
            deltaTime = (currentTime - lastTimestamp) / 1000.0; // convert ms to seconds
        }
        lastTimestamp = currentTime; // store for next iteration

        // Clamp dt to [5 ms, 100 ms] to avoid numerical issues from very short or
        // very long gaps between control loop iterations.
        deltaTime = Math.max(Math.min(deltaTime, 0.1), 0.005);

        // ---- Integral ----
        integral += error * deltaTime; // accumulate error × time
        // Anti-windup: clamp so the integral can't grow unboundedly.
        integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));

        // ---- Derivative with low-pass filter ----
        double rawDerivative = (error - lastError) / deltaTime; // rate of change of error
        // Exponential moving average: blends previous smoothed derivative with raw one.
        double derivative = (1 - filter) * lastDerivative + filter * rawDerivative;
        lastDerivative = derivative; // save for next iteration
        lastError = error;           // save current error for next iteration

        // ---- Feedforward ----
        // A small constant in the direction of the error to overcome stiction.
        double feedforward = Math.signum(error) * kF;

        // ---- Combined PIDF output ----
        double power = kP * error + kI * integral + kD * derivative + feedforward;

        // Clamp output to valid motor power range.
        return Math.max(-1, Math.min(1, power));
    }
}