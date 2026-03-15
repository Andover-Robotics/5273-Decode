package org.firstinspires.ftc.teamcode.auto.utils;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

/**
 * Paths is a utility class that builds Road Runner trajectory sequences for
 * full autonomous routines.
 *
 * <p>{@link #buildPath} constructs a single chained trajectory that:
 * <ol>
 *   <li>Drives to the obelisk scanning position to read the motif tag.</li>
 *   <li>Drives to the shooting position and performs an outtake.</li>
 *   <li>Iterates over every intake row (2 to 0) and all artifact columns within
 *       each row, collecting up to three artifacts per row.</li>
 *   <li>Drives to the parking zone at the end of the autonomous period.</li>
 * </ol>
 *
 * <p>Many intermediate steps (obelisk scan, shoot with lock, individual intake
 * cycles) are currently commented out while the path structure is being tuned.
 * These are preserved in comments so they can be re-enabled as subsystems are
 * validated.
 *
 * <p><b>Coordinate system:</b> all positions are Road Runner field-frame inches,
 * with X pointing right and Y pointing forward.
 */
public class Paths {

    /**
     * Builds the full autonomous path as a single chained Road Runner
     * {@link Action} that can be passed directly to
     * {@code Actions.runBlocking()}.
     *
     * <p>Path outline:
     * <ol>
     *   <li>Start at {@code startPose}.</li>
     *   <li>Strafe (spline heading) to the obelisk scan position.</li>
     *   <li>Strafe to the shooting position and optionally perform a shoot action.</li>
     *   <li>For each intake row (2 down to 0):
     *     <ul>
     *       <li>Strafe to {@code artifactPositions[row][0]} at {@code intakeHeading}.</li>
     *       <li>Strafe to each subsequent artifact position in the row.</li>
     *     </ul>
     *   </li>
     *   <li>Strafe to the park position at {@code outtakeHeading}.</li>
     * </ol>
     *
     * @param mecanumDrive       the Road Runner drive used to build trajectories
     * @param actions            action factory for inline action injection (currently unused but reserved)
     * @param startPose          field pose at which the path begins
     * @param shootPos           2-D field position where the robot shoots
     * @param parkPos            2-D field position of the parking zone
     * @param obeliskScanPos     2-D field position where the Limelight can see the obelisk tag
     * @param artifactPositions  3x4 array: index[row][0] = row start, index[row][1..3] = artifact positions
     * @param intakeHeading      robot heading (radians) while traversing the intake rows
     * @param outtakeHeading     robot heading (radians) at the shooting and parking positions
     * @param obeliskScanHeading robot heading (radians) when scanning the obelisk tag
     * @return the complete autonomous trajectory as a single {@link Action}
     */
    public static Action buildPath(
            MecanumDrive mecanumDrive,
            BotActions actions,
            Pose2d startPose,
            Vector2d shootPos,
            Vector2d parkPos,
            Vector2d obeliskScanPos,
            Vector2d[][] artifactPositions,
            double intakeHeading,
            double outtakeHeading,
            double obeliskScanHeading
    ) {
        // Start building the trajectory chain from the provided starting pose.
        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(startPose);

        // Drive to the obelisk scan position so the Limelight can see the motif tag.
        // (The scan action is currently commented out while being validated.)
        builder.strafeToSplineHeading(obeliskScanPos, obeliskScanHeading)
                /*.stopAndAdd(actions.actionScanObelisk())*/; // enable when obelisk scan is ready

        // Drive to the shooting position and execute the shoot-with-lock action.
        // (Shoot action is commented out; trajectory movement is active.)
        builder.strafeToSplineHeading(shootPos, outtakeHeading)
               /*.stopAndAdd(actions.actionShootWithLock(actions.aprilTag.getObeliskId(), 2.0, mecanumDrive))*/;

        // Iterate from the farthest intake row (row 2) back to the nearest (row 0).
        for (int row = 2; row >= 0; row--) {
            // Move to the starting position of this row at the intake heading.
            builder.strafeToSplineHeading(
                    artifactPositions[row][0],  // first waypoint of the row
                    intakeHeading               // heading that points intake toward the artifacts
            );

            // Traverse each artifact column within the row (columns 1, 2, 3).
            for (int col = 1; col <= 3; col++) {
                // Strafe to each artifact's field position to collect it.
                builder.strafeToSplineHeading(
                               artifactPositions[row][col], // x,y of this artifact
                               intakeHeading                // keep same intake heading throughout
                       );
            }

            // After collecting all artifacts in this row, drive to shoot.
            // (Disabled; re-enable when sorted outtake is validated.)
            //builder.stopAndAdd(actions.actionShootWithLock(actions.aprilTag.getObeliskId(), 2.0, mecanumDrive));
        }

        // After all intake rows, drive to the parking zone and stop.
        return builder
                .strafeToSplineHeading(parkPos, outtakeHeading) // drive to park position
                //.stopAndAdd(actions.actionPark()) // enable when parking action is ready
                .build(); // finalise and return the trajectory action
    }
}
