package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
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
 * FasterRedClose — Red Alliance, Close-Side Autonomous WITHOUT Motif (No Obelisk Color Matching)
 *
 * <p><b>Alliance:</b> Red &nbsp;|&nbsp; <b>Starting Position:</b> Close side, facing
 * field-forward at 0° (positive-X direction).</p>
 *
 * <h2>Mission Overview</h2>
 * <p>This is the simpler, faster variant of the red close-side autonomous. Unlike the "Motif"
 * variants, this OpMode does NOT scan the Obelisk AprilTag or rotate the indexer drum to
 * match required colors. Instead it fires whatever artifact is currently loaded in slot "two"
 * unconditionally after each intake cycle. This removes ~1–2 seconds of drum rotation and
 * confirmation time per shot, allowing faster cycle times at the cost of color-matching:
 * <ol>
 *   <li>Initializes the indexer to slot "two" and spins the flywheel to {@link #SHOOT_RPM} RPM
 *       during the initial drive to the shooting position.</li>
 *   <li>Fires the first pre-loaded artifact after a 1.65 s delay.</li>
 *   <li>Performs three intake sweeps (rows at Y ≈ 50, 76.5, and 96 in.) and returns to shoot
 *       after each, firing immediately on arrival without any color rotation.</li>
 *   <li>Parks in the designated zone.</li>
 * </ol>
 * </p>
 *
 * <h2>Key Differences from Motif Variants</h2>
 * <ul>
 *   <li><b>No {@code rotateToMotifColorBeforeOuttake}</b> calls anywhere — the drum stays
 *       in whatever position it was loaded/left in, maximising speed.</li>
 *   <li><b>No {@code actionScanObelisk}</b> — the Limelight is not used at all.</li>
 *   <li><b>Lower {@link #SHOOT_RPM}</b> (3240 vs. 3580) — tuned for the fixed drum position
 *       rather than variable drum positions.</li>
 *   <li><b>{@link #SHOOT_HEADING_DEG} = -142.5°</b> — slightly different aim angle optimized
 *       for the no-rotation drum position.</li>
 *   <li><b>Short {@link #timeUntilStartOuttake}</b> (1.65 s) — just enough for flywheel
 *       spinup; no time reserved for drum rotation.</li>
 *   <li><b>Offset direction flipped for rows 2 and 3</b>: {@link #INTAKE2_START_OFFSET_X}
 *       and {@link #INTAKE3_START_OFFSET_X} are positive and <em>added</em> to
 *       {@link #INTAKE_START_X} (vs. subtracted in motif variants), shifting the start point
 *       outward (toward negative X) to better cover this field layout's artifact positions.</li>
 *   <li><b>Shallower dodgeGate angle</b> (140°) with a -4 in. Y offset (vs. 160° / +4 in.
 *       in the standard motif), reflecting a slightly different gate geometry or path shape.</li>
 * </ul>
 *
 * @see FasterRedCloseMotif  Motif (color-matching) version for the red close side
 * @see FasterBlueClose      Blue alliance mirror of this autonomous
 * @see BotActions           Subsystem action factory
 * @see Hardware             Hardware initialization
 */
@Config
@Autonomous(name = "Faster Red Auto No Motif", group = "Autonomous")
public class FasterRedClose extends LinearOpMode {

    /**
     * Maximum translational velocity (in/s) during intake corridor sweeps.
     * 30 in/s prioritises speed — no color matching is needed, so the robot can afford
     * faster transit without worrying about losing time to drum rotation.
     */
    public static double maxIntakeDrivingVel = 30;

    /** X-coordinate (inches) of the shooting position. */
    public static double SHOOT_X = 17;

    /** Y-coordinate (inches) of the shooting position. */
    public static double SHOOT_Y = 40;

    /**
     * Heading (degrees) of the robot at the shooting position.
     * At -142.5° the robot aims toward the scoring targets from the fixed drum position
     * (no rotation means the artifact exits at a predictable angle every time).
     */
    public static double SHOOT_HEADING_DEG = -142.5;

    /**
     * Heading offset (degrees) applied after the first shot. Set to 0° — since there is no
     * drum rotation drift, the same heading is equally accurate for all subsequent shots.
     */
    public static double SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT = 0;

    /** X-coordinate (inches) of the intake sweep start for all rows. */
    public static double INTAKE_START_X = 12;

    /**
     * X offset (inches) ADDED to {@link #INTAKE_START_X} for row 2.
     * Note: this is added (not subtracted) — +3.0 shifts the start outward toward negative X,
     * widening coverage for row-2 artifacts in this field configuration.
     */
    public static double INTAKE2_START_OFFSET_X = 3.0;

    /**
     * X offset (inches) ADDED to {@link #INTAKE_START_X} for row 3.
     * Same additive convention as row 2; +5.0 provides the widest row-3 start position.
     */
    public static double INTAKE3_START_OFFSET_X = 5.0;

    /**
     * X-coordinate (inches) at the end of the row-1 intake sweep (toward the field wall).
     */
    public static double INTAKE_END_X = -16;

    /**
     * Additional depth (inches) subtracted from {@link #INTAKE_END_X} for rows 2 and 3,
     * giving a deeper sweep to cover all artifacts in later rows.
     */
    public static double intake_END_2And3_XOffset = 6.0;

    /** Y-coordinate (inches) of the first intake row. */
    public static double INTAKE1_Y = 50;

    /** Y-coordinate (inches) of the second intake row. */
    public static double INTAKE2_Y = 76.5;

    /** Y-coordinate (inches) of the third intake row. */
    public static double INTAKE3_Y = 96;

    /** X-coordinate (inches) of the gate obstacle reference. */
    public static double gate_X = -12;

    /** Y-coordinate (inches) of the gate obstacle reference. */
    public static double gate_Y = 60;

    /**
     * Dwell time (seconds) at the gate position for the optional {@code goToGate} action.
     * {@code goToGate} is defined but commented out in the active sequence.
     */
    public static double gateWaitTime = 1.5;

    /** X-coordinate (inches) of the final parking position. */
    public static double PARK_X = 6;

    /** Y-coordinate (inches) of the final parking position. */
    public static double PARK_Y = 68;

    /**
     * Target flywheel speed in RPM.
     * 3240 RPM — lower than the motif variants (3580) and calibrated for the fixed drum
     * position used in this no-rotation variant.
     */
    public static int SHOOT_RPM = 3240;

    /**
     * Delay (seconds) before triggering the outtake sequence.
     * At 1.65 s this is short — just enough for flywheel spinup and robot settle.
     * No extra time is reserved for drum rotation (there is none in this variant).
     * Subsequent shots: backToShoot2 adds +0.5 s; backToShoot3 adds +1.0 s to accommodate
     * the longer return paths from rows 2 and 3.
     */
    public static double timeUntilStartOuttake = 1.65; // Time until you start the outtake action, which still includes the wait for actuator

    /**
     * Initializes hardware, pre-compiles trajectories, and runs the full no-motif routine.
     *
     * <p><b>Execution Order:</b>
     * <ol>
     *   <li>{@code toShoot} — Drive to shooting pose; set indexer to slot 2; fire artifact 1.</li>
     *   <li>{@code intake1} — Sweep row 1 (Y ≈ 50 in.) at 30 in/s.</li>
     *   <li>{@code backToShoot1} — Return to shooting pose; fire artifact 2 immediately.</li>
     *   <li>{@code intake2} — Sweep row 2 (Y ≈ 76.5 in.).</li>
     *   <li>{@code backToShoot2} — Navigate around gate via dodgeGate; fire artifact 3 (+0.5 s).</li>
     *   <li>{@code intake3} — Sweep row 3 (Y ≈ 96 in.).</li>
     *   <li>{@code backToShoot3} — Return to shooting pose; fire artifact 4 (+1.0 s).</li>
     *   <li>{@code toPark} — Drive to parking zone.</li>
     * </ol>
     * {@code actionPeriodic} runs as a persistent parallel thread for telemetry/sensor maintenance.
     * No localizer re-seed is performed here (unlike the motif variants) since the no-motif
     * start pose is also (0, 0, 0°) and no pre-match movement is expected.</p>
     */
    @Override
    public void runOpMode() {
        // Robot starts at origin facing 0° (positive-X / field-forward direction).
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        // Initialize all hardware subsystems: drive, indexer, intake, flywheel, camera.
        Hardware hardware = new Hardware(hardwareMap, telemetry, this, startPose);
        // BotActions provides Road Runner Action wrappers for every subsystem command.
        BotActions botActions = hardware.actions;
        // MecanumDrive handles mecanum-wheel trajectory following.
        MecanumDrive drive = hardware.mecanumDrive;

        // Shooting pose: (17, 40, -142.5°). The heading is tuned for the fixed drum slot's
        // artifact exit angle — no rotation means every shot has the same geometry.
        Pose2d shootingPose = new Pose2d(
                SHOOT_X,
                SHOOT_Y,
                Math.toRadians(SHOOT_HEADING_DEG)
        );

        // Row-1 intake corridor: sweeps from X=12 to X=-16 at Y=50, heading = 180°.
        Pose2d intake1PoseStart = new Pose2d(INTAKE_START_X, INTAKE1_Y, Math.toRadians(180));
        Pose2d intake1PoseEnd = new Pose2d(INTAKE_END_X, INTAKE1_Y, Math.toRadians(180));
        // Gate obstacle at (-12, 60, -90°). The -90° heading reflects the gate's orientation
        // relative to the robot's approach from below (row 1 end position).
        Pose2d gate = new Pose2d(gate_X, gate_Y, Math.toRadians(-90));

        // Row-2 intake corridor: start ADDED by +3 (outward to -X), end 6 in. deeper.
        // Note the additive offset convention here — the start shifts toward negative X,
        // which is toward the field wall (further from the shooting position).
        Pose2d intake2PoseStart = new Pose2d(INTAKE_START_X + INTAKE2_START_OFFSET_X, INTAKE2_Y, Math.toRadians(180));
        Pose2d intake2PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE2_Y, Math.toRadians(180));
        // dodgeGate: +14 in. in X and -4 in. in Y from row-2 end, heading = 140°.
        // The -4 in. Y (toward start) combined with +14 in. X creates a diagonal arc that
        // steers the robot away from the gate before the final spline toward the shooting pose.
        Pose2d dodgeGate = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset + 14, INTAKE2_Y - 4, Math.toRadians(140));

        // Row-3 intake corridor: start ADDED by +5, same end depth as row 2.
        Pose2d intake3PoseStart = new Pose2d(INTAKE_START_X + INTAKE3_START_OFFSET_X, INTAKE3_Y, Math.toRadians(180));
        Pose2d intake3PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE3_Y, Math.toRadians(180));

        // Parking pose at (6, 68, 180°).
        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(180));

        // toShoot: Drives from start to the shooting pose while simultaneously setting the
        // indexer to slot 2, spinning up the flywheel, and firing after the 1.65 s delay.
        // Unlike the motif variants, initializeAuto is called HERE (not in a separate start
        // action) because there is no toObelisk step preceding this.
        Action toShoot = new ParallelAction(
                // Single-segment spline-heading trajectory from start (0,0,0°) to shooting pose.
                drive.actionBuilder(startPose)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading)
                        .build(),

                // Spin flywheel to target RPM during the drive.
                botActions.actionStartOuttake(SHOOT_RPM),
                // Set the indexer drum to slot 2 (pre-loaded artifact position) during the drive.
                botActions.initializeAuto(Indexer.IndexerState.two),

                new SequentialAction(
                        // Wait 1.65 s for robot to arrive and flywheel to reach SHOOT_RPM.
                        new SleepAction(timeUntilStartOuttake),
                        // Fire the first artifact — no drum rotation, just open the gate.
                        botActions.actionQuickOuttake()
                )
        );

        // intake1: Feedback-driven sweep of row 1 (Y ≈ 50 in.) at 30 in/s.
        Action intake1 = botActions.actionIntakeThreeFeedback(shootingPose, intake1PoseStart, intake1PoseEnd, drive, maxIntakeDrivingVel);

        // goToGate: Optional route through the gate before returning to shoot after row 1.
        // Defined but currently unused (commented out in the run sequence).
        Action goToGate = drive.actionBuilder(intake1PoseEnd)
                .strafeToLinearHeading(gate.position, gate.heading)
                // Pause at the gate for gateWaitTime = 1.5 s if this action is ever enabled.
                .waitSeconds(gateWaitTime)
                .build();

        // backToShoot1: Returns from row-1 end directly to the shooting pose and fires
        // the second artifact. No dodge needed for row 1 (it's below the gate obstacle).
        Action backToShoot1 = new ParallelAction(
                // Direct spline-heading return from row-1 end to shooting pose.
                drive.actionBuilder(intake1PoseEnd) // goToGate
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Re-spin the flywheel (it coasted during the intake sweep).
                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        // Same 1.65 s wait — short drive back from row 1.
                        new SleepAction(timeUntilStartOuttake),
                        // Fire the second artifact immediately on arrival (no color rotation).
                        botActions.actionQuickOuttake()
                )
        );

        // intake2: Feedback-driven sweep of row 2 (Y ≈ 76.5 in.) at 30 in/s.
        Action intake2 = botActions.actionIntakeThreeFeedback(shootingPose, intake2PoseStart, intake2PoseEnd, drive, maxIntakeDrivingVel);

        // backToShoot2: Two-segment return from row-2 end that arcs around the gate obstacle
        // via dodgeGate (140°), then spline-heads to the shooting pose. Fires artifact 3.
        // +0.5 s extra wait to account for the longer two-segment path.
        Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        // Arc through dodgeGate to clear the gate (angled at 140°, -4 in. Y).
                        .strafeToSplineHeading(dodgeGate.position, dodgeGate.heading)
                        // Then spline-heading into the shooting pose.
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Flywheel spinup in parallel with the two-segment drive.
                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        // 2.15 s (1.65 + 0.5) — extra buffer for the longer two-segment return.
                        new SleepAction(timeUntilStartOuttake + 0.5),
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

        // backToShoot3: Direct spline-heading return from row-3 end to the shooting pose.
        // Row 3 is far enough from the gate (Y=96 vs. gate Y=60) that no dodge is needed.
        // +1.0 s extra for the longest return drive of the routine.
        Action backToShoot3 = new ParallelAction(
                // Single-segment spline-heading return.
                drive.actionBuilder(intake3PoseEnd)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Flywheel spinup in parallel.
                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        // 2.65 s (1.65 + 1.0) — longest wait for the most distant return trip.
                        new SleepAction(timeUntilStartOuttake + 1.0),
                        // Fire the fourth and final artifact.
                        botActions.actionQuickOuttake()
                )
        );

        // toPark: Final spline from the shooting pose to the parking zone for bonus points.
        Action toPark = drive.actionBuilder(shootingPose)
                .strafeToSplineHeading(
                        parkPose.position,  // Target: (6, 68)
                        parkPose.heading    // Re-orient to 180°
                )
                .build();

        // Wait for the Drive Station "Play" button.
        waitForStart();
        // Exit if emergency-stopped before the match.
        if (isStopRequested()) return;

        // Execute the full routine. runBlocking holds the thread until all actions complete.
        Actions.runBlocking(
                // Outer ParallelAction: two concurrent threads.
                new ParallelAction(
                        // Thread 1 — Periodic: telemetry and sensor maintenance.
                        botActions.actionPeriodic(),
                        // Thread 2 — Main sequential mission.
                        new SequentialAction(
                                toShoot,        // Drive to shoot; init indexer + flywheel; fire artifact 1
                                intake1,        // Sweep row 1 (Y ≈ 50 in.)
                                /*goToGate,*/   // (Disabled) Optional gate approach
                                backToShoot1,   // Return to shoot; fire artifact 2
                                intake2,        // Sweep row 2 (Y ≈ 76.5 in.)
                                backToShoot2,   // Dodge gate; return to shoot; fire artifact 3
                                intake3,        // Sweep row 3 (Y ≈ 96 in.)
                                backToShoot3,   // Return to shoot; fire artifact 4
                                toPark          // Drive to parking zone
                        )
                )
        );
    }
}