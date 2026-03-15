package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

/**
 * AntiDefense provides a pose-hold (position lock) capability for the robot.
 *
 * <p>When a defending robot makes contact and tries to push this robot off its
 * intended position, AntiDefense counteracts the push by computing the error
 * between the robot's current pose and a saved reference pose, then commanding
 * the drive motors with a proportional correction signal to drive back toward
 * that reference.
 *
 * <p>Usage:
 * <ol>
 *   <li>Call {@link #lock(Pose2d)} when the robot reaches the position it should
 *       hold (e.g., just before starting the shooting sequence).</li>
 *   <li>Call {@link #update(Pose2d)} every loop iteration to apply the
 *       correction.  If the error is within tolerance the robot holds still;
 *       otherwise proportional drive powers push it back.</li>
 *   <li>Call {@link #unlock()} to release the pose lock when done.</li>
 * </ol>
 */
public class AntiDefense {

    // -----------------------------------------------------------------------
    // Dashboard-tunable gains and tolerances
    // -----------------------------------------------------------------------

    /** Proportional gain for X/Y position correction.
     *  Multiplied by the XY error vector to produce a drive velocity.
     *  Increase to push back harder; too high causes oscillation. */
    public static double kP = 0.0;

    /** Heading (angular) proportional gain.
     *  Multiplied by the heading error (radians) to produce a yaw correction. */
    public static double kH = 0.0; // heading gain

    /** Maximum XY distance (inches) from the reference pose before correction
     *  activates.  Within this tolerance the robot is considered "on target"
     *  and no drive output is applied. */
    public static double xyTolerance = 0.5;

    /** Maximum heading error (radians) before rotational correction activates.
     *  π/30 ≈ 6° – small enough to keep the robot well-aimed. */
    public static double aTolerance = Math.PI / 30;

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------

    /** The Road Runner MecanumDrive used to command wheel powers. */
    MecanumDrive drive;

    /** The pose the robot should return to if it is pushed away.
     *  {@code null} when the lock is inactive. */
    private Pose2d referencePoint;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Creates an AntiDefense instance.
     *
     * @param telemetry currently unused; reserved for future debug output
     * @param drive     the MecanumDrive used to issue correction velocities
     */
    public AntiDefense(Telemetry telemetry, MecanumDrive drive)
    {
        this.drive = drive; // store the drive reference for update() to use
    }

    // -----------------------------------------------------------------------
    // Lock control
    // -----------------------------------------------------------------------

    /**
     * Activates the pose lock at the given position.
     * From this point on, {@link #update(Pose2d)} will try to return the robot
     * to {@code currentBotPos} whenever it is pushed away.
     *
     * @param currentBotPos the field-relative pose to hold (usually the robot's
     *                      current estimated pose at the moment of locking)
     */
    public void lock(Pose2d currentBotPos)
    {
        referencePoint = currentBotPos; // save this pose as the correction target
    }

    /**
     * Deactivates the pose lock.
     * After calling this, {@link #update(Pose2d)} is a no-op until
     * {@link #lock(Pose2d)} is called again.
     */
    public void unlock()
    {
        referencePoint = null; // null reference means "no lock active"
    }

    // -----------------------------------------------------------------------
    // Helper
    // -----------------------------------------------------------------------

    /**
     * Wraps an angle (radians) into the range (-π, π].
     *
     * <p>Road Runner's pose representation already normalises headings, but
     * this helper is needed after {@code Pose2d.minusExp()} which may return
     * a heading outside the standard range.
     *
     * @param angle heading error in radians
     * @return equivalent angle in (-π, π]
     */
    private double clampAngle(double angle) {
        angle = angle % (2 * Math.PI);            // reduce to (-2π, 2π)
        if (angle > Math.PI)  angle -= 2 * Math.PI;  // fold the [π, 2π) half
        if (angle < -Math.PI) angle += 2 * Math.PI;  // fold the (-2π, -π] half
        return angle;
    }

    // -----------------------------------------------------------------------
    // Update loop
    // -----------------------------------------------------------------------

    /**
     * Must be called every loop iteration while the lock is active.
     * Computes the pose error between the reference point and the current
     * robot position, and if the error exceeds the tolerances commands
     * proportional correction drive powers.
     *
     * <p>If the lock is not active ({@link #referencePoint} is {@code null})
     * this method returns immediately without issuing any drive command.
     *
     * @param currentPos the robot's current estimated field-relative pose,
     *                   typically from the Road Runner localizer
     */
    public void update(Pose2d currentPos)
    {
        // Do nothing when the lock is not engaged.
        if(referencePoint == null) return;

        // Compute the spatial error: reference – current.
        // minusExp() performs a pose-relative subtraction that correctly
        // accounts for heading, giving the error in the robot's local frame.
        Pose2d error = referencePoint.minusExp(currentPos);

        // Extract the translational (XY) and rotational (heading) components.
        Vector2d xyError = error.position;
        double theta = clampAngle(error.heading.toDouble()); // wrap heading error to (-π, π]

        // If the robot is already within both tolerance bands, no correction needed.
        if(xyError.norm() < xyTolerance && Math.abs(theta) < aTolerance) return;

        // Command a proportional velocity in the robot's local frame:
        // - XY: position error scaled by kP gives a velocity vector pointing toward the reference.
        // - Yaw: heading error scaled by kH gives a rotation rate to restore the heading.
        drive.setDrivePowers(new PoseVelocity2d(error.position.times(kP), theta * kH));
    }
}
