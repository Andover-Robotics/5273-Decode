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
 * FasterRedCloseMotif15 — Red Alliance, Close-Side Autonomous with Motif Scanning, "15-Ball" Tuning
 *
 * <p><b>Alliance:</b> Red &nbsp;|&nbsp; <b>Starting Position:</b> Close side, facing
 * field-forward at 0° (positive-X direction).</p>
 *
 * <h2>Mission Overview</h2>
 * <p>This is a high-throughput variant of the red close-side motif autonomous, tuned to
 * score as many artifacts as possible (up to 15 balls across all three intake rows) within
 * the 30-second autonomous period:
 * <ol>
 *   <li>Initializes the indexer drum to slot "two" and spins the flywheel to
 *       {@link #SHOOT_RPM} RPM.</li>
 *   <li>Drives directly to the shooting position and fires the first pre-loaded artifact
 *       immediately after a {@link #timeUntilStartOuttake} delay (no color rotation needed
 *       for the first shot — see note below).</li>
 *   <li>Simultaneously scans the Obelisk AprilTag during the initial drive so color data
 *       is ready by the time subsequent shots are needed.</li>
 *   <li>Performs three intake sweeps (rows at Y ≈ 48.5, 76, and 96 in.) and returns to
 *       shoot after each, rotating the indexer drum to the motif-correct color before
 *       each subsequent shot (slots 1, 2, 3).</li>
 *   <li>Parks in the designated zone.</li>
 * </ol>
 * </p>
 *
 * <h2>Key Differences from {@link FasterRedCloseMotif} (Standard Motif)</h2>
 * <ul>
 *   <li><b>Start heading 0°</b> (facing positive X) instead of 180° — the robot is placed
 *       differently on this side of the field for this tuning configuration.</li>
 *   <li><b>Faster intake velocity</b>: {@link #maxIntakeDrivingVel} = 30 in/s (vs. 15),
 *       prioritising cycle speed over per-pickup accuracy.</li>
 *   <li><b>No color rotation before the first shot</b>: The commented-out line
 *       {@code rotateToMotifColorBeforeOuttake(0, ...)} in {@code toShoot} is disabled with
 *       the note "no need in 12 ball or more" — at high cycle counts the time spent rotating
 *       for the pre-loaded first artifact costs more than it gains.</li>
 *   <li><b>Longer {@link #timeUntilStartOuttake}</b> (3.0 s vs. 2.5 s) to account for the
 *       direct single-segment trajectory to the shooting pose taking longer than the
 *       two-step intermediate approach used by the standard variant.</li>
 *   <li><b>Obelisk scanning embedded in {@code toShoot}</b> instead of the outer
 *       ParallelAction, because this variant's outer loop does not include
 *       {@code actionScanObelisk} at the top level.</li>
 *   <li><b>Category 0</b> motif rotation (vs. category 2) — a faster rotation mode that
 *       does not wait for position confirmation, trading certainty for speed.</li>
 *   <li><b>Shallower intake end</b>: {@link #INTAKE_END_X} = -13.5 in. (vs. -16.5 in.),
 *       and {@link #INTAKE1_Y} = 48.5 in. (vs. 52 in.) — slightly adjusted artifact
 *       positions for this field/robot configuration.</li>
 *   <li><b>Extended post-rotation waits</b>: subsequent shots use
 *       {@code timeUntilStartOuttake + 1}, {@code + 2}, {@code + 1} respectively to allow
 *       extra settle time after the longer return drives.</li>
 *   <li>A {@code goToGate} action is defined for potential use (routing through the gate
 *       obstacle on the row-1 return) but is commented out of the active sequence.</li>
 * </ul>
 *
 * <h2>Coordinate System</h2>
 * <p>All positions in inches relative to start pose (0, 0, 0°). The robot faces positive-X
 * (field-forward) at the start. Positive Y runs perpendicular toward the far field wall.
 * The shooting position is at (17, 40) with a -140° heading.</p>
 *
 * @see FasterRedCloseMotif      Standard red close motif autonomous (slower, more accurate)
 * @see FasterRedClose           Red close autonomous with no motif color matching
 * @see BotActions               Subsystem action factories
 * @see Hardware                 Hardware initialization and subsystem wiring
 */
@Config
@Autonomous(name = "Faster Red Auto With Motif 15 Ball", group = "Autonomous")
public class FasterRedCloseMotif15 extends LinearOpMode {

    /**
     * Maximum translational velocity (in/s) while driving through intake corridors.
     * Set to 30 in/s (double the standard motif variant) to maximize cycle speed and
     * collect the most artifacts possible within the 30-second autonomous window.
     * The trade-off is slightly reduced pickup reliability compared to slower speeds.
     */
    public static double maxIntakeDrivingVel = 30;

    /**
     * X-coordinate (inches) of the Obelisk (Motif) AprilTag structure.
     * Used in the {@code toObelisk} action that is currently commented out in the sequence,
     * and as context for the Limelight scan embedded inside {@code toShoot}.
     */
    public static double OBELISK_X = 8;

    /**
     * Y-coordinate (inches) of the Obelisk (Motif) structure.
     * See {@link #OBELISK_X} for context.
     */
    public static double OBELISK_Y = 36;

    /**
     * Heading (degrees) the robot faces while pointing at the Obelisk AprilTag.
     * At -60° the Limelight camera has an optimal sightline to the tag.
     */
    public static double OBELISK_HEADING_DEG = -60;

    /**
     * X-coordinate (inches) of the shooting position.
     * Same as the standard motif variant; positioned for a clear shot to all scoring targets.
     */
    public static double SHOOT_X = 17;

    /**
     * Y-coordinate (inches) of the shooting position.
     */
    public static double SHOOT_Y = 40;

    /**
     * Heading (degrees) of the robot at the shooting position.
     * At -140° (slightly less aggressive than the standard motif's -135°), tuned to balance
     * aim angle and the trajectory approach smoothness for this faster configuration.
     */
    public static double SHOOT_HEADING_DEG = -140;

    /**
     * Small heading offset (degrees) added to {@link #SHOOT_HEADING_DEG} for all shots after
     * the first. +2° nudges the robot slightly counter-clockwise to compensate for accumulated
     * localization drift across multiple intake-and-return cycles.
     */
    public static double SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT = 2; // -2

    /**
     * X-coordinate (inches) of the intake sweep start for all rows.
     * The robot begins sweeping from this point and drives toward negative X to collect artifacts.
     */
    public static double INTAKE_START_X = 12;

    /**
     * Inward X offset (inches) subtracted from {@link #INTAKE_START_X} for row 2.
     * Prevents re-collecting row-1 artifacts by shifting the start point inward.
     */
    public static double INTAKE2_START_OFFSET_X = 3.0;

    /**
     * Inward X offset (inches) subtracted from {@link #INTAKE_START_X} for row 3.
     * Larger shift for the deepest artifact row.
     */
    public static double INTAKE3_START_OFFSET_X = 5.0;

    /**
     * X-coordinate (inches) at the end of the first intake sweep.
     * Shallower than the standard motif variant (-13.5 vs. -16.5), reflecting this tuning's
     * field/placement configuration or a narrower artifact spread.
     */
    public static double INTAKE_END_X = -13.5;

    /**
     * Additional depth (inches) subtracted from {@link #INTAKE_END_X} for rows 2 and 3.
     * Artifacts in later rows are deeper into the field and require a longer sweep.
     */
    public static double intake_END_2And3_XOffset = 8;

    /**
     * Y-coordinate (inches) of the first intake row.
     * At 48.5 in. this is slightly closer to start than the standard variant (52 in.),
     * shaving a small amount of transit time for the faster 15-ball tuning.
     */
    public static double INTAKE1_Y = 48.5;

    /**
     * Y-coordinate (inches) of the second intake row (mid-field).
     */
    public static double INTAKE2_Y = 76;

    /**
     * Y-coordinate (inches) of the third intake row (near far wall).
     */
    public static double INTAKE3_Y = 96;

    /**
     * X-coordinate (inches) of the field gate obstacle used in path planning.
     * Referenced by the optional {@code goToGate} action and the {@code dodgeGate} waypoint.
     */
    public static double gate_X = -12;

    /**
     * Y-coordinate (inches) of the field gate obstacle.
     */
    public static double gate_Y = 60;

    /**
     * Time in seconds the robot waits at the gate position if the {@code goToGate} action is
     * enabled. Currently not used in the active sequence (goToGate is commented out), but
     * preserved for testing alternate gate-approach strategies.
     */
    public static double gateWaitTime = 1.5;

    /**
     * X-coordinate (inches) of the final parking position.
     */
    public static double PARK_X = 6;

    /**
     * Y-coordinate (inches) of the final parking position.
     */
    public static double PARK_Y = 68;

    /**
     * Target flywheel speed in RPM.
     * 3580 RPM provides the power needed to reliably reach the scoring targets regardless
     * of drum rotation position, same as the standard motif variant.
     */
    public static int SHOOT_RPM = 3580;

    /**
     * Base delay (seconds) before triggering the outtake sequence inside each parallel action.
     * At 3.0 s this is longer than the standard motif's 2.5 s because this variant drives
     * directly to the shooting pose in a single segment (no intermediate waypoint), and the
     * single-segment trajectory to (17, 40, -140°) takes slightly more time from (0, 0, 0°).
     * Subsequent shots add additional time: backToShoot1 uses {@code +1}, backToShoot2 uses
     * {@code +2}, and backToShoot3 uses {@code +1} to handle the longer return paths.
     */
    public static double timeUntilStartOuttake = 3.0; // Time until you start the outtake action, which still includes the wait for actuator

    // bunch of compensations for bad rr
    // has quick outtake and quick intake

    /**
     * Main autonomous routine. Pre-compiles all trajectories and action sequences before
     * {@code waitForStart}, then runs the full 15-ball routine via {@code Actions.runBlocking}.
     *
     * <p><b>Execution Order:</b>
     * <ol>
     *   <li>{@code start} — Initialize indexer to slot 2 and spin flywheel.</li>
     *   <li>{@code toShoot} — Drive to shooting pose (single-segment spline); scan obelisk
     *       in parallel; fire first artifact immediately after the 3.0 s delay.</li>
     *   <li>{@code intake1} — Sweep row 1 (Y ≈ 48.5 in.) at 30 in/s.</li>
     *   <li>{@code backToShoot1} — Return to shoot; rotate for slot 1; fire artifact 2.</li>
     *   <li>{@code intake2} — Sweep row 2 (Y ≈ 76 in.).</li>
     *   <li>{@code backToShoot2} — Navigate around gate; rotate for slot 2; fire artifact 3.</li>
     *   <li>{@code intake3} — Sweep row 3 (Y ≈ 96 in.).</li>
     *   <li>{@code backToShoot3} — Return; rotate for slot 3; fire artifact 4.</li>
     *   <li>{@code toPark} — Drive to parking zone.</li>
     * </ol>
     * {@code actionPeriodic} runs as a persistent parallel thread throughout.</p>
     */
    @Override
    public void runOpMode() {
        // Robot starts at origin facing 0° (positive-X direction, toward the field interior
        // from the red close side in this placement configuration).
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        // Initialize all robot hardware subsystems (drive, indexer, intake, flywheel, camera).
        Hardware hardware = new Hardware(hardwareMap, telemetry, this, startPose);

        // BotActions provides composable Road Runner Action objects for every subsystem command.
        BotActions botActions = hardware.actions;

        // MecanumDrive handles mecanum-wheel trajectory following with Road Runner's motion profiles.
        MecanumDrive drive = hardware.mecanumDrive;

        // Obelisk pose for reference and the disabled toObelisk action.
        // The scan is embedded in toShoot for this variant instead.
        Pose2d obeliskPose = new Pose2d(
                OBELISK_X,
                OBELISK_Y,
                Math.toRadians(OBELISK_HEADING_DEG)
        );

        // Shooting pose: (17, 40, -140°). The slightly shallower heading (-140° vs -135°)
        // was tuned to improve trajectory smoothness for the direct single-segment approach.
        Pose2d shootingPose = new Pose2d(
                SHOOT_X,
                SHOOT_Y,
                Math.toRadians(SHOOT_HEADING_DEG)
        );

        // Row-1 intake corridor (Y ≈ 48.5 in.): sweeps from X=12 to X=-13.5 facing 180°.
        Pose2d intake1PoseStart = new Pose2d(INTAKE_START_X, INTAKE1_Y, Math.toRadians(180));
        Pose2d intake1PoseEnd = new Pose2d(INTAKE_END_X, INTAKE1_Y, Math.toRadians(180));

        // Gate obstacle reference pose (used in the optional goToGate action below).
        Pose2d gate = new Pose2d(gate_X, gate_Y, Math.toRadians(90));

        // Row-2 intake corridor (Y ≈ 76 in.): start shifted 3 in. inward, end 8 in. deeper.
        Pose2d intake2PoseStart = new Pose2d(INTAKE_START_X - INTAKE2_START_OFFSET_X, INTAKE2_Y, Math.toRadians(180));
        Pose2d intake2PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE2_Y, Math.toRadians(180));

        // dodgeGate: intermediate waypoint to steer around the gate on the row-2 return.
        // +12 in. in X and +4 in. in Y from the row-2 end, heading = 180° (straight return).
        // The 180° heading (vs. 160° in the standard motif) reflects the different trajectory
        // shape needed when approaching the shooting pose from this starting heading.
        Pose2d dodgeGate = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset + 12, INTAKE2_Y + 4, Math.toRadians(180));

        // Row-3 intake corridor (Y ≈ 96 in.): deepest position, 5 in. inward start offset.
        Pose2d intake3PoseStart = new Pose2d(INTAKE_START_X - INTAKE3_START_OFFSET_X, INTAKE3_Y, Math.toRadians(180));
        Pose2d intake3PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE3_Y, Math.toRadians(180));

        // Parking pose at (6, 68, 180°) — end position for the autonomous parking bonus.
        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(180));

        // start: Pre-match initialization — set indexer to slot 2 and begin flywheel spinup.
        // These run in parallel so the flywheel reaches RPM while the indexer positions itself.
        // Remove actionStartOuttake in toShoot when adding this
        Action start = new ParallelAction(
                // Rotate the indexer drum to the pre-loaded "two" position.
                botActions.initializeAuto(Indexer.IndexerState.two),
                // Start spinning the flywheel toward SHOOT_RPM immediately.
                botActions.actionStartOuttake(SHOOT_RPM)
        );

        // remove soon
        // toObelisk: Alternative first step that drives to the obelisk for a direct tag scan.
        // Currently disabled — the scan is now embedded inside toShoot via actionScanObelisk.
        // Kept as a reference for restoring the explicit obelisk visit if needed.
        Action toObelisk = new ParallelAction(
                drive.actionBuilder(startPose)
                        .strafeToSplineHeading(obeliskPose.position, obeliskPose.heading)
                        .build(),

                botActions.initializeAuto(Indexer.IndexerState.two),
                botActions.actionStartOuttake(SHOOT_RPM),
                botActions.actionScanObelisk()
        );

        // toShoot: Single-segment spline to the shooting pose while scanning the obelisk and
        // spinning up the flywheel. Unlike the standard motif variant, this fires the first
        // artifact WITHOUT a color rotation (the rotation call is commented out with the note
        // "no need in 12 ball or more" — at high cycle counts, skipping the first rotation
        // saves time that is more valuable than color-matching the pre-loaded artifact).
        Action toShoot = new ParallelAction(
                // Direct single-segment spline-heading path from start (0,0,0°) to shooting pose.
                // No intermediate waypoint is needed here because starting at 0° allows a smooth
                // single arc into the -140° shooting heading.
                drive.actionBuilder(startPose) // obeliskPose
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading)
                        .build(),

                // Actively scan the obelisk AprilTag during the drive so color data is ready
                // for slots 1, 2, 3 which DO use rotateToMotifColorBeforeOuttake.
                botActions.actionScanObelisk(), // With new cam pose
                // Maintain flywheel at target RPM during transit.
                botActions.actionStartOuttake(SHOOT_RPM),
                //botActions.rotateToMotifColorBeforeOuttake(0, botActions::getObeliskId, 0), // - no need in 12 ball or more

                new SequentialAction(
                        // Wait 3.0 s for the robot to arrive and the flywheel to reach full speed.
                        new SleepAction(timeUntilStartOuttake),
                        // Fire the first (pre-loaded) artifact without color matching.
                        botActions.actionQuickOuttake()
                )
        );

        // intake1: Feedback-driven sweep of row 1 (Y ≈ 48.5 in.) at 30 in/s.
        Action intake1 = botActions.actionIntakeThreeFeedback(shootingPose, intake1PoseStart, intake1PoseEnd, drive, maxIntakeDrivingVel);

        // goToGate: Optional route through the gate obstacle before returning to shoot after row 1.
        // Drives to the gate at (−12, 60, 90°), waits gateWaitTime = 1.5 s there, then the
        // caller would need to add the return-to-shoot segment. Currently commented out in the
        // main sequence in favour of the direct backToShoot1 return.
        Action goToGate = drive.actionBuilder(intake1PoseEnd)
                .strafeToLinearHeading(gate.position, gate.heading)
                // Pause at the gate position for the configured dwell time.
                .waitSeconds(gateWaitTime)
                .build();

        // backToShoot1: Returns from row-1 end to the shooting pose and fires the second artifact.
        // Uses category 0 rotation (faster, no wait for confirmation) to save time.
        // The +4 s total wait (timeUntilStartOuttake + 1 = 4.0 s) gives extra buffer for
        // the longer flywheel re-spinup after being idle during the intake sweep.
        Action backToShoot1 = new ParallelAction(
                // Direct spline-heading return from row-1 end to shooting pose with heading offset.
                drive.actionBuilder(intake1PoseEnd) // goToGate
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Spin flywheel back up to target RPM while driving.
                botActions.actionStartOuttake(SHOOT_RPM),
                // Rotate drum to the motif-correct color for slot 1 (category 0 = no wait).
                botActions.rotateToMotifColorBeforeOuttake(1, botActions::getObeliskId, 0),

                new SequentialAction(
                        // Wait 4.0 s (timeUntilStartOuttake + 1) for arrival and flywheel stabilization.
                        new SleepAction(timeUntilStartOuttake + 1),
                        // Fire the second artifact.
                        botActions.actionQuickOuttake()
                )
        );

        // intake2: Feedback-driven sweep of row 2 (Y ≈ 76 in.) at 30 in/s.
        Action intake2 = botActions.actionIntakeThreeFeedback(shootingPose, intake2PoseStart, intake2PoseEnd, drive, maxIntakeDrivingVel);

        // backToShoot2: Returns from row-2 end, navigating around the gate via dodgeGate,
        // and fires the third artifact. The +2 s extra wait (5.0 s total) accounts for the
        // longer two-segment path from the deepest row-2 position.
        Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        // Strafe to the dodgeGate intermediate waypoint to clear the gate obstacle.
                        .strafeTo(dodgeGate.position)
                        // Then spline-heading into the shooting pose.
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Flywheel spinup concurrently with the two-segment drive.
                botActions.actionStartOuttake(SHOOT_RPM),
                // Rotate drum to motif-correct color for slot 2 (category 0).
                botActions.rotateToMotifColorBeforeOuttake(2, botActions::getObeliskId, 0),

                new SequentialAction(
                        // Wait 5.0 s (timeUntilStartOuttake + 2) — extra time for the longer path.
                        new SleepAction(timeUntilStartOuttake + 2.0),
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

        // backToShoot3: Returns from row-3 end directly to the shooting pose (no gate dodge
        // needed — row 3 is far enough from the gate that the direct spline avoids it).
        // Fires the fourth and final artifact.
        Action backToShoot3 = new ParallelAction(
                // Direct single-segment spline-heading return from row-3 end.
                drive.actionBuilder(intake3PoseEnd)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Flywheel spinup in parallel.
                botActions.actionStartOuttake(SHOOT_RPM),
                // Rotate drum to motif-correct color for slot 3 (category 0).
                botActions.rotateToMotifColorBeforeOuttake(3, botActions::getObeliskId, 0),

                new SequentialAction(
                        // Wait 4.0 s (timeUntilStartOuttake + 1) for arrival and spinup.
                        new SleepAction(timeUntilStartOuttake + 1.0),
                        // Fire the fourth and final artifact.
                        botActions.actionQuickOuttake()
                )
        );

        // toPark: Final trajectory from the shooting pose to the parking zone.
        Action toPark = drive.actionBuilder(shootingPose)
                .strafeToSplineHeading(
                        parkPose.position,   // Target: (6, 68)
                        parkPose.heading     // Re-orient to 180°
                )
                .build();

        // Block until the Drive Station starts the match.
        waitForStart();
        // Exit immediately if the OpMode was stopped before execution (e.g., emergency stop).
        if (isStopRequested()) return;

        // Execute the full autonomous routine. runBlocking drives the OpMode thread until
        // every action finishes or the OpMode is externally terminated.
        Actions.runBlocking(
                // Outer ParallelAction provides two concurrent threads:
                new ParallelAction(
                        // Thread 1 — Periodic maintenance: telemetry updates and sensor refreshes.
                        botActions.actionPeriodic(),

                        // Thread 2 — Main sequential mission routine.
                        new SequentialAction(
                                // Re-seed the Road Runner localizer with the exact known start pose,
                                // correcting any pose drift between hardware init and match start.
                                new InstantAction(() -> drive.localizer.setPose(startPose)),

                                start,          // Set indexer slot 2, spin flywheel
                                //toObelisk,    // (Disabled) Drive to obelisk explicitly
                                toShoot,        // Drive to shooting pose; scan obelisk; fire artifact 1
                                intake1,        // Sweep row 1 (Y ≈ 48.5 in.)
                                /*goToGate,*/   // (Disabled) Optional gate approach from row 1
                                backToShoot1,   // Return to shoot; fire artifact 2 (slot 1)
                                intake2,        // Sweep row 2 (Y ≈ 76 in.)
                                backToShoot2,   // Dodge gate; return to shoot; fire artifact 3 (slot 2)
                                intake3,        // Sweep row 3 (Y ≈ 96 in.)
                                backToShoot3,   // Return to shoot; fire artifact 4 (slot 3)
                                toPark          // Drive to parking zone
                        )
                )
        );
    }
}
