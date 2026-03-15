package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

/**
 * FasterRedCloseMotifLinear — Red Alliance, Close-Side Autonomous with Motif Scanning,
 * using Linear Heading Interpolation throughout.
 *
 * <p><b>Alliance:</b> Red &nbsp;|&nbsp; <b>Starting Position:</b> Close side, facing
 * field-forward at 0° (positive-X direction).</p>
 *
 * <h2>What "Linear Heading" Means</h2>
 * <p>Road Runner offers two heading interpolation strategies along a trajectory:
 * <ul>
 *   <li><b>Spline heading</b> ({@code strafeToSplineHeading}): heading changes smoothly
 *       following a cubic spline, which can produce very smooth arcs but may result in
 *       the robot facing unexpected directions mid-path if not carefully constrained.</li>
 *   <li><b>Linear heading</b> ({@code strafeToLinearHeading}): heading changes at a constant
 *       rate proportional to the distance traveled. The robot turns steadily and predictably
 *       from its current heading to the target heading. This is simpler to reason about and
 *       produces more consistent final orientations, at the cost of less fluid motion.</li>
 * </ul>
 * This variant uses {@code strafeToLinearHeading} (and {@code strafeToLinearHeading}) for
 * <em>all</em> driving segments, making heading transitions more deterministic. It was likely
 * developed to test whether linear interpolation improves shooting accuracy by ensuring the
 * robot reliably ends at the exact heading needed for the shot.</p>
 *
 * <h2>Unique Characteristics vs. Other Motif Variants</h2>
 * <ul>
 *   <li><b>Visits the Obelisk explicitly</b>: Unlike the other red motif variants, this one
 *       actually runs {@code toObelisk} (drives to the obelisk pose) in the active sequence
 *       before proceeding to {@code toShoot}. The {@code toShoot} trajectory therefore
 *       starts from {@code obeliskPose} rather than {@code startPose}.</li>
 *   <li><b>No {@code start} action in the run sequence</b>: {@code start} is defined but
 *       commented out ({@code //start,}); initialization is done inside {@code toObelisk}.</li>
 *   <li><b>Category 0 rotations</b>: All {@code rotateToMotifColorBeforeOuttake} calls use
 *       category 0 (fast, no confirmation wait), unlike FasterRedCloseMotif which uses 2.</li>
 *   <li><b>Very short {@link #timeUntilStartOuttake}</b> (1.0 s) — the shortest of all motif
 *       variants. Because the robot explicitly visits the obelisk first and the shooting
 *       trajectory starts from there, less wait time is needed for the flywheel to spin up
 *       (it was already spinning during the obelisk visit).</li>
 *   <li><b>SHOOT_HEADING_OFFSET = 0°</b>: No heading correction is needed between shots,
 *       suggesting linear interpolation provides sufficient final heading accuracy.</li>
 *   <li><b>SHOOT_RPM = 3480</b>: Slightly lower than the other motif variants (3580),
 *       fine-tuned for this heading configuration.</li>
 *   <li><b>Two-segment {@code backToShoot2}</b>: Uses two {@code strafeToLinearHeading}
 *       segments (dodgeGate then shooting pose) rather than {@code strafeTo + splineHeading}.</li>
 * </ul>
 *
 * <h2>Execution Sequence</h2>
 * <p>start pose (0,0,0°) → toObelisk → toShoot (from obeliskPose) → [intake/shoot ×3] → park</p>
 *
 * @see FasterRedCloseMotif       Standard motif variant (spline heading, 180° start)
 * @see FasterRedCloseMotif15     High-throughput motif variant (no explicit obelisk visit)
 * @see BotActions                Subsystem action factory
 * @see Hardware                  Hardware initialization
 */
@Config
@Autonomous(name = "Faster Red Auto With Motif Linear Heading", group = "Autonomous")
public class FasterRedCloseMotifLinear extends LinearOpMode {

    /**
     * Maximum translational velocity (in/s) during intake driving segments.
     * 30 in/s matches the 15-ball variant — prioritising speed for high cycle count.
     */
    public static double maxIntakeDrivingVel = 30;

    /**
     * X-coordinate (inches) of the Obelisk (Motif) structure.
     * This is actually visited in the active sequence ({@code toObelisk} is not commented out
     * in the {@code runBlocking} call), so the robot physically drives here first.
     */
    public static double OBELISK_X = 8;

    /** Y-coordinate (inches) of the Obelisk structure. */
    public static double OBELISK_Y = 36;

    /**
     * Heading (degrees) the robot faces at the obelisk to optimally view the AprilTag
     * with the Limelight camera.
     */
    public static double OBELISK_HEADING_DEG = -60;

    /** X-coordinate (inches) of the shooting position. */
    public static double SHOOT_X = 17;

    /** Y-coordinate (inches) of the shooting position. */
    public static double SHOOT_Y = 40;

    /**
     * Heading (degrees) of the robot at the shooting position.
     * At -145° this is the steepest clockwise rotation of any red motif variant, tuned
     * specifically for the approach from the obelisk pose using linear heading interpolation.
     */
    public static double SHOOT_HEADING_DEG = -145;

    /**
     * Heading offset (degrees) applied after the first shot. Set to 0° — linear heading
     * interpolation provides sufficient accuracy that no correction offset is needed.
     */
    public static double SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT = 0; // -2

    /** X-coordinate (inches) of the intake sweep start. */
    public static double INTAKE_START_X = 12;

    /** Inward X offset (inches) for the row-2 sweep start position. */
    public static double INTAKE2_START_OFFSET_X = 3.0;

    /** Inward X offset (inches) for the row-3 sweep start position. */
    public static double INTAKE3_START_OFFSET_X = 5.0;

    /**
     * X-coordinate (inches) at the end of the row-1 intake sweep.
     * Same depth as the 15-ball variant (-13.5 in.) rather than the standard motif (-16.5 in.).
     */
    public static double INTAKE_END_X = -13.5;

    /** Additional depth (inches) for the row-2 and row-3 intake end positions. */
    public static double intake_END_2And3_XOffset = 8;

    /** Y-coordinate (inches) of the first intake row (same as 15-ball at 48.5 in.). */
    public static double INTAKE1_Y = 48.5;

    /** Y-coordinate (inches) of the second intake row. */
    public static double INTAKE2_Y = 76;

    /** Y-coordinate (inches) of the third intake row. */
    public static double INTAKE3_Y = 96;

    /** X-coordinate (inches) of the gate obstacle reference. */
    public static double gate_X = -12;

    /** Y-coordinate (inches) of the gate obstacle reference. */
    public static double gate_Y = 60;

    /**
     * Dwell time (seconds) at the gate position when {@code goToGate} is used.
     * Currently the action is commented out in the active sequence.
     */
    public static double gateWaitTime = 1.5;

    /** X-coordinate (inches) of the final parking position. */
    public static double PARK_X = 6;

    /** Y-coordinate (inches) of the final parking position. */
    public static double PARK_Y = 68;

    /**
     * Target flywheel speed in RPM.
     * 3480 RPM — slightly lower than the other motif variants (3580) and fine-tuned for
     * the specific geometry of this variant's linear-heading approach to the shooting pose.
     */
    public static int SHOOT_RPM = 3480;

    /**
     * Base delay (seconds) before triggering the outtake sequence.
     * At 1.0 s this is the shortest of all motif variants. Because the robot visits the
     * obelisk first and the flywheel is already spinning during that transit, it is closer
     * to target RPM when {@code toShoot} begins — less spinup time is needed.
     * Subsequent shots: backToShoot1/3 use {@code +0} (same 1.0 s); backToShoot2 uses {@code +1.0}.
     */
    public static double timeUntilStartOuttake = 1.0; // Time until you start the outtake action, which still includes the wait for actuator

    // bunch of compensations for bad rr
    // has quick outtake and quick intake

    /**
     * Main autonomous routine. Pre-compiles all trajectories and actions, then executes
     * the full sequence after {@code waitForStart}.
     *
     * <p><b>Key difference from other variants:</b> This routine actually runs {@code toObelisk}
     * as the first driving step (it is not commented out in the {@code SequentialAction} inside
     * {@code runBlocking}). The {@code toShoot} action therefore starts its trajectory from
     * {@code obeliskPose} rather than {@code startPose}.</p>
     *
     * <p><b>Execution Order:</b>
     * <ol>
     *   <li>{@code toObelisk} — Drive to the obelisk; scan the AprilTag; initialize indexer;
     *       spin up flywheel. (start action is commented out — init happens here.)</li>
     *   <li>{@code toShoot} — Linear-heading drive from obelisk to shooting pose; rotate
     *       indexer for slot 0; fire first artifact.</li>
     *   <li>{@code intake1} — Sweep row 1 at 30 in/s.</li>
     *   <li>{@code backToShoot1} — Linear-heading return; rotate for slot 1; fire artifact 2.</li>
     *   <li>{@code intake2} — Sweep row 2.</li>
     *   <li>{@code backToShoot2} — Two-segment linear-heading return via dodgeGate;
     *       rotate for slot 2; fire artifact 3.</li>
     *   <li>{@code intake3} — Sweep row 3.</li>
     *   <li>{@code backToShoot3} — Linear-heading return; rotate for slot 3; fire artifact 4.</li>
     *   <li>{@code toPark} — Drive to parking zone.</li>
     * </ol>
     * {@code actionPeriodic} runs as a persistent parallel thread throughout.</p>
     */
    @Override
    public void runOpMode() {
        // Robot starts at the field origin facing 0° (positive-X direction, field-forward).
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        // Initialize all hardware subsystems.
        Hardware hardware = new Hardware(hardwareMap, telemetry, this, startPose);

        // BotActions wraps subsystem commands into composable Road Runner Actions.
        BotActions botActions = hardware.actions;

        // MecanumDrive with Road Runner trajectory following.
        MecanumDrive drive = hardware.mecanumDrive;

        // Obelisk pose: the robot actually drives here in this variant (toObelisk is active)
        // to let the Limelight get a clear read of the field's Motif AprilTag before shooting.
        Pose2d obeliskPose = new Pose2d(
                OBELISK_X,
                OBELISK_Y,
                Math.toRadians(OBELISK_HEADING_DEG)
        );

        // Shooting pose: (17, 40, -145°). The approach from the obelisk (at ~8, 36, -60°)
        // to this pose is handled with a single linear-heading segment in toShoot.
        Pose2d shootingPose = new Pose2d(
                SHOOT_X,
                SHOOT_Y,
                Math.toRadians(SHOOT_HEADING_DEG)
        );

        // Row-1 intake corridor: sweeps from X=12 to X=-13.5 at Y=48.5, heading = 180°.
        Pose2d intake1PoseStart = new Pose2d(INTAKE_START_X, INTAKE1_Y, Math.toRadians(180));
        Pose2d intake1PoseEnd = new Pose2d(INTAKE_END_X, INTAKE1_Y, Math.toRadians(180));

        // Gate obstacle reference: used in the optional goToGate action and for the
        // dodgeGate computation below.
        Pose2d gate = new Pose2d(gate_X, gate_Y, Math.toRadians(90));

        // Row-2 intake corridor: offset start, deeper end.
        Pose2d intake2PoseStart = new Pose2d(INTAKE_START_X - INTAKE2_START_OFFSET_X, INTAKE2_Y, Math.toRadians(180));
        Pose2d intake2PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE2_Y, Math.toRadians(180));

        // dodgeGate: intermediate waypoint for the row-2 return path.
        // +12 in. in X and +4 in. in Y from the row-2 end, maintaining 180° heading
        // (a simpler linear dodge compared to the angled approach of other variants).
        Pose2d dodgeGate = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset + 12, INTAKE2_Y + 4, Math.toRadians(180));

        // Row-3 intake corridor: furthest from start (Y ≈ 96 in.).
        Pose2d intake3PoseStart = new Pose2d(INTAKE_START_X - INTAKE3_START_OFFSET_X, INTAKE3_Y, Math.toRadians(180));
        Pose2d intake3PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE3_Y, Math.toRadians(180));

        // Parking pose: final destination after all shots are complete.
        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(180));

        // start: Defined but commented out in the run sequence — initialization is handled
        // by toObelisk which includes initializeAuto and actionStartOuttake.
        // Remove actionStartOuttake in toShoot when adding this
        Action start = new ParallelAction(
                // Set indexer to pre-loaded slot 2.
                botActions.initializeAuto(Indexer.IndexerState.two),
                // Begin flywheel spinup.
                botActions.actionStartOuttake(SHOOT_RPM)
        );

        // remove soon
        // toObelisk: ACTIVE in this variant — the robot drives to the obelisk pose using
        // linear heading interpolation. This ensures the Limelight has a clear, stable view
        // of the AprilTag before the shooting sequence begins.
        // initializeAuto and actionStartOuttake run in parallel with the drive so the indexer
        // and flywheel are ready by the time the robot reaches the obelisk.
        Action toObelisk = new ParallelAction(
                // Linear-heading drive from start to the obelisk pose.
                // strafeToLinearHeading produces a constant angular rate turn from 0° to -60°
                // while following the translational path.
                drive.actionBuilder(startPose)
                        .strafeToLinearHeading(obeliskPose.position, obeliskPose.heading)
                        .build(),

                // Initialize the indexer drum to slot 2 while driving.
                botActions.initializeAuto(Indexer.IndexerState.two),
                // Spin the flywheel to target RPM while driving.
                botActions.actionStartOuttake(SHOOT_RPM),
                // Scan the Obelisk AprilTag during the approach — by the time the robot
                // reaches the obelisk and then the shooting pose, the tag ID will be decoded.
                botActions.actionScanObelisk()
        );

        // toShoot: Linear-heading drive FROM the obelisk pose TO the shooting pose,
        // while rotating the indexer for slot 0 and firing the first artifact.
        // Starts from obeliskPose (not startPose) because toObelisk runs before this.
        Action toShoot = new ParallelAction(
                // Single linear-heading segment from obelisk to shooting position.
                // Heading linearly interpolates from -60° to -145° over the distance traveled.
                drive.actionBuilder(obeliskPose)
                        .strafeToLinearHeading(shootingPose.position, shootingPose.heading)
                        .build(),

                // Maintain flywheel at target RPM during transit (already at RPM from toObelisk).
                botActions.actionStartOuttake(SHOOT_RPM),
                // Rotate indexer to motif-correct color for slot 0 (category 0 = fast, no wait).
                botActions.rotateToMotifColorBeforeOuttake(0, botActions::getObeliskId, 0), // - no need in 12 ball or more

                new SequentialAction(
                        // Only 1.0 s wait needed since flywheel was already spinning during toObelisk.
                        new SleepAction(timeUntilStartOuttake),
                        // Fire the first (pre-loaded) artifact.
                        botActions.actionQuickOuttake()
                )
        );

        // intake1: Feedback-driven sweep of row 1 (Y ≈ 48.5 in.) at 30 in/s.
        Action intake1 = botActions.actionIntakeThreeFeedback(shootingPose, intake1PoseStart, intake1PoseEnd, drive, maxIntakeDrivingVel);

        // goToGate: Optional route through the gate obstacle. Defined but not used in the
        // active sequence (commented out with /*goToGate,*/ in runBlocking).
        Action goToGate = drive.actionBuilder(intake1PoseEnd)
                .strafeToLinearHeading(gate.position, gate.heading)
                // Pause at gate for gateWaitTime seconds.
                .waitSeconds(gateWaitTime)
                .build();

        // backToShoot1: Linear-heading return from row-1 end to shooting pose.
        // Rotates indexer for slot 1 in parallel (category 0), fires the second artifact.
        Action backToShoot1 = new ParallelAction(
                // Linear-heading drive from row-1 end to shooting pose (0° offset = exact heading).
                drive.actionBuilder(intake1PoseEnd) // goToGate
                        .strafeToLinearHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Flywheel spinup in parallel.
                botActions.actionStartOuttake(SHOOT_RPM),
                // Rotate drum for slot 1 color (category 0 = no confirmation wait).
                botActions.rotateToMotifColorBeforeOuttake(1, botActions::getObeliskId, 0),

                new SequentialAction(
                        // 1.0 s wait before firing (short since flywheel was kept warm).
                        new SleepAction(timeUntilStartOuttake),
                        // Fire the second artifact.
                        botActions.actionQuickOuttake()
                )
        );

        // intake2: Feedback-driven sweep of row 2 (Y ≈ 76 in.) at 30 in/s.
        Action intake2 = botActions.actionIntakeThreeFeedback(shootingPose, intake2PoseStart, intake2PoseEnd, drive, maxIntakeDrivingVel);

        // backToShoot2: Two-segment linear-heading return from row-2 end via dodgeGate.
        // Uses strafeToLinearHeading for both segments (unique to this linear variant —
        // other variants use strafeTo + strafeToSplineHeading for the same dodge maneuver).
        Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        // Segment 1: Linear-heading drive to dodgeGate to clear the gate obstacle.
                        .strafeToLinearHeading(dodgeGate.position, dodgeGate.heading)
                        // Segment 2: Linear-heading drive from dodgeGate into the shooting pose.
                        .strafeToLinearHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Flywheel spinup in parallel with the two-segment drive.
                botActions.actionStartOuttake(SHOOT_RPM),
                // Rotate drum for slot 2 color.
                botActions.rotateToMotifColorBeforeOuttake(2, botActions::getObeliskId, 0),

                new SequentialAction(
                        // +1.0 s extra (2.0 s total) for the longer two-segment path from row 2.
                        new SleepAction(timeUntilStartOuttake + 1.0),
                        // Fire the third artifact.
                        botActions.actionQuickOuttake()
                )
        );

        /*Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        .setTangent(Math.toRadians(180))

                        .splineToSplineHeading(
                                shootingPose,
                                Math.toRadians(20)
                        )
                        .build(),

                botActions.actionStartOuttake(SHOOT_RPM),
                botActions.rotateToMotifColorBeforeOuttake(2, botActions::getObeliskId, 0),

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake + 0.5),
                        botActions.actionQuickOuttake()
                )
        );*/

        // intake3: Feedback-driven sweep of row 3 (Y ≈ 96 in.) at 30 in/s.
        Action intake3 = botActions.actionIntakeThreeFeedback(shootingPose, intake3PoseStart, intake3PoseEnd, drive, maxIntakeDrivingVel);

        // backToShoot3: Linear-heading return from row-3 end directly to the shooting pose.
        // No gate dodge needed — the direct path from Y=96 clears the gate at Y=60.
        Action backToShoot3 = new ParallelAction(
                // Single linear-heading segment from row-3 end to shooting pose.
                drive.actionBuilder(intake3PoseEnd)
                        .strafeToLinearHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Flywheel spinup in parallel.
                botActions.actionStartOuttake(SHOOT_RPM),
                // Rotate drum for slot 3 color (final shot).
                botActions.rotateToMotifColorBeforeOuttake(3, botActions::getObeliskId, 0),

                new SequentialAction(
                        // +1.0 s extra (2.0 s total) to account for the longer transit from row 3.
                        new SleepAction(timeUntilStartOuttake + 1.0),
                        // Fire the fourth and final artifact.
                        botActions.actionQuickOuttake()
                )
        );

        // toPark: Linear-heading drive from the shooting pose to the parking zone.
        Action toPark = drive.actionBuilder(shootingPose)
                .strafeToLinearHeading(
                        parkPose.position,  // Target coordinates: (6, 68)
                        parkPose.heading    // Final heading: 180°
                )
                .build();

        // Wait for the Drive Station "Play" button.
        waitForStart();
        // Exit if emergency-stopped before the match begins.
        if (isStopRequested()) return;

        // Execute the full autonomous sequence, blocking until complete.
        Actions.runBlocking(
                // Outer ParallelAction: two concurrent threads.
                new ParallelAction(
                        // Thread 1 — Periodic maintenance: telemetry and sensor updates.
                        botActions.actionPeriodic(),

                        // Thread 2 — Main sequential mission routine.
                        new SequentialAction(
                                // Re-seed the localizer with the exact start pose to eliminate
                                // any drift accumulated between hardware init and match start.
                                new InstantAction(() -> drive.localizer.setPose(startPose)),

                                //start,        // (Disabled) Separate init step — handled by toObelisk
                                toObelisk,      // Drive to obelisk; init indexer + flywheel; scan tag
                                toShoot,        // Linear-heading drive to shooting pose; fire artifact 1
                                intake1,        // Sweep row 1 (Y ≈ 48.5 in.)
                                /*goToGate,*/   // (Disabled) Optional gate approach from row 1
                                backToShoot1,   // Return to shoot; fire artifact 2 (slot 1)
                                intake2,        // Sweep row 2 (Y ≈ 76 in.)
                                backToShoot2,   // Two-segment dodge-gate return; fire artifact 3 (slot 2)
                                intake3,        // Sweep row 3 (Y ≈ 96 in.)
                                backToShoot3,   // Return to shoot; fire artifact 4 (slot 3)
                                toPark          // Drive to parking zone
                        )
                )
        );
    }
}

