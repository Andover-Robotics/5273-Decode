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
 * FasterBlueClose — Blue Alliance, Close-Side Autonomous WITHOUT Motif (No Color Matching)
 *
 * <p><b>Alliance:</b> Blue &nbsp;|&nbsp; <b>Starting Position:</b> Close side, starting at
 * (0, 0, 0°) — facing positive-X (field-forward).</p>
 *
 * <h2>Mission Overview</h2>
 * <p>This is the blue alliance mirror of {@link FasterRedClose}. The robot:
 * <ol>
 *   <li>Drives to the blue-side shooting position at (-17, 40, -50°) while initializing the
 *       indexer drum to slot "two" and spinning the flywheel. Fires the first artifact after
 *       a 1.65 s delay.</li>
 *   <li>Performs three intake sweeps toward the positive-X field wall (opposite to red side),
 *       returning to shoot after each without any color rotation.</li>
 *   <li>Parks in the designated zone.</li>
 * </ol>
 * </p>
 *
 * <h2>Key Mirror Differences from FasterRedClose</h2>
 * <ul>
 *   <li><b>Shooting pose is in negative-X territory</b>: X = -17 (vs. +17 for red), heading
 *       = -50° (vs. -142.5°). The blue alliance shooting position is on the opposite side of
 *       the field, so the heading flips by ~90° to face the same scoring targets.</li>
 *   <li><b>Intake directions are mirrored</b>: {@link #INTAKE_START_X} = -12 (starts toward
 *       negative X), {@link #INTAKE_END_X} = +16 (sweeps toward positive X / blue field wall).
 *       All corridor headings are 0° (facing positive X) instead of 180°.</li>
 *   <li><b>Gate is at positive X</b>: {@link #gate_X} = +12 (vs. -12 for red). The gate
 *       obstacle is on the opposite side of the field on blue. Gate heading = -90° (same
 *       as red, pointing downfield).</li>
 *   <li><b>dodgeGate shifts NEGATIVE</b>: dodge is -14 in. in X (vs. +14 for red) and -4
 *       in. Y, reflecting the mirrored geometry. Heading = +40° (vs. 140° for red).</li>
 *   <li><b>Parking at (-6, 68, 0°)</b>: X = -6 (vs. +6 for red), heading = 0°.</li>
 *   <li><b>Lower intake velocity</b>: {@link #maxIntakeDrivingVel} = 17 in/s (vs. 30 in/s for
 *       red). This blue variant uses a more conservative speed, possibly due to field
 *       differences or artifact placement on the blue side.</li>
 *   <li><b>Same timing</b>: {@link #SHOOT_RPM} = 3240, {@link #timeUntilStartOuttake} = 1.65 s.</li>
 *   <li><b>No motif scanning</b>: Like its red counterpart, no Limelight scanning or drum
 *       rotation occurs — the robot fires from whatever position the drum is in.</li>
 * </ul>
 *
 * @see FasterRedClose       Red alliance mirror of this autonomous
 * @see FasterBlueCloseMotif Blue alliance version with Obelisk color matching
 * @see BotActions           Subsystem action factory
 * @see Hardware             Hardware initialization
 */
@Config
@Autonomous(name = "Faster Blue Auto No Motif", group = "Autonomous")
public class FasterBlueClose extends LinearOpMode {

    /**
     * Maximum translational velocity (in/s) during intake corridor sweeps.
     * 17 in/s is more conservative than the red counterpart's 30 in/s, providing better
     * artifact pickup reliability on the blue side.
     */
    public static double maxIntakeDrivingVel = 17;

    /**
     * X-coordinate (inches) of the blue shooting position.
     * Negative because the blue alliance starting position is on the left (negative-X) side.
     */
    public static double SHOOT_X = -17;

    /** Y-coordinate (inches) of the blue shooting position. */
    public static double SHOOT_Y = 40;

    /**
     * Heading (degrees) of the robot at the blue shooting position.
     * -50° faces the scoring targets from the negative-X side of the field, roughly mirroring
     * the red side's -142.5° around the field centerline.
     */
    public static double SHOOT_HEADING_DEG = -50;

    /**
     * Heading offset (degrees) applied after the first shot. 0° — no correction needed
     * since there is no drum rotation to accumulate drift from.
     */
    public static double SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT = 0; // -2

    /**
     * X-coordinate (inches) of the intake sweep start (blue side, negative-X territory).
     */
    public static double INTAKE_START_X = -12;

    /**
     * X offset (inches) ADDED to {@link #INTAKE_START_X} for row 2.
     * Value is negative (-3.0), so adding it shifts start outward toward positive X
     * (the blue field wall direction), widening row-2 coverage.
     */
    public static double INTAKE2_START_OFFSET_X = -3.0;

    /**
     * X offset (inches) ADDED to {@link #INTAKE_START_X} for row 3.
     * Value is negative (-5.0), providing the widest row-3 start position.
     */
    public static double INTAKE3_START_OFFSET_X = -5.0;

    /**
     * X-coordinate (inches) at the end of the row-1 intake sweep (toward the blue field wall,
     * positive X direction). Positive value because blue sweeps toward positive X.
     */
    public static double INTAKE_END_X = 16;

    /**
     * Additional depth (inches) for rows 2 and 3. Negative value (-6.0) is subtracted from
     * {@link #INTAKE_END_X}, but in blue's mirrored coordinate system the subtraction produces
     * a MORE positive result (e.g. 16 - (-6) = 22), pushing deeper into the field.
     */
    public static double intake_END_2And3_XOffset = -6.0;

    /** Y-coordinate (inches) of the first intake row. */
    public static double INTAKE1_Y = 50;

    /** Y-coordinate (inches) of the second intake row. */
    public static double INTAKE2_Y = 76.5;

    /** Y-coordinate (inches) of the third intake row. */
    public static double INTAKE3_Y = 96;

    /**
     * X-coordinate (inches) of the gate obstacle on the blue side (positive X, field right).
     */
    public static double gate_X = 12;

    /** Y-coordinate (inches) of the gate obstacle. */
    public static double gate_Y = 60;

    /** Dwell time at gate if goToGate action is enabled. Currently unused (commented out). */
    public static double gateWaitTime = 1.5;

    /** X-coordinate (inches) of the blue parking position (negative X side). */
    public static double PARK_X = -6;

    /** Y-coordinate (inches) of the blue parking position. */
    public static double PARK_Y = 68;

    /**
     * Target flywheel speed in RPM. Same as the red no-motif variant (3240).
     */
    public static int SHOOT_RPM = 3240;

    /**
     * Delay (seconds) before triggering outtake. 1.65 s matches the red no-motif variant —
     * just enough for flywheel spinup without any drum rotation wait.
     */
    public static double timeUntilStartOuttake = 1.65; // Time until you start the outtake action, which still includes the wait for actuator

    /**
     * Initializes hardware, pre-compiles all trajectories, and runs the blue no-motif routine.
     *
     * <p>All trajectories mirror the red no-motif variant geometrically: intake corridors sweep
     * toward positive X (blue field wall), the shooting pose is at negative X, and the gate
     * dodge arc angles in the opposite direction. The timing parameters are identical to the
     * red variant.</p>
     *
     * <p><b>Execution Order:</b>
     * <ol>
     *   <li>{@code toShoot} — Drive to blue shooting pose (-17, 40, -50°); init drum + flywheel;
     *       fire artifact 1 after 1.65 s.</li>
     *   <li>{@code intake1} — Sweep row 1 (Y ≈ 50 in.) toward positive X at 17 in/s.</li>
     *   <li>{@code backToShoot1} — Return to shoot; fire artifact 2.</li>
     *   <li>{@code intake2} — Sweep row 2 (Y ≈ 76.5 in.).</li>
     *   <li>{@code backToShoot2} — Navigate around blue-side gate; fire artifact 3 (+0.5 s).</li>
     *   <li>{@code intake3} — Sweep row 3 (Y ≈ 96 in.).</li>
     *   <li>{@code backToShoot3} — Return to shoot; fire artifact 4 (+1.0 s).</li>
     *   <li>{@code toPark} — Drive to blue parking zone.</li>
     * </ol>
     * </p>
     */
    @Override
    public void runOpMode() {
        // Robot starts at origin facing 0° (positive-X direction, blue close side).
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        // Initialize all hardware subsystems.
        Hardware hardware = new Hardware(hardwareMap, telemetry, this, startPose);
        // BotActions wraps subsystem commands as composable Road Runner Actions.
        BotActions botActions = hardware.actions;
        // MecanumDrive for trajectory following.
        MecanumDrive drive = hardware.mecanumDrive;

        // Blue shooting pose: (-17, 40, -50°). On the negative-X (left) side of the field,
        // heading -50° faces the scoring targets from the blue alliance position.
        Pose2d shootingPose = new Pose2d(
                SHOOT_X,
                SHOOT_Y,
                Math.toRadians(SHOOT_HEADING_DEG)
        );

        // Row-1 intake corridor: sweeps from X=-12 to X=+16 at Y=50, heading = 0°.
        // Blue intake sweeps TOWARD positive X (the right/blue field wall), opposite to red.
        Pose2d intake1PoseStart = new Pose2d(INTAKE_START_X, INTAKE1_Y, Math.toRadians(0));
        Pose2d intake1PoseEnd = new Pose2d(INTAKE_END_X, INTAKE1_Y, Math.toRadians(0));
        // Gate on blue side at (+12, 60, -90°). Positive X reflects the blue-side gate location.
        Pose2d gate = new Pose2d(gate_X, gate_Y, Math.toRadians(-90));

        // Row-2 intake corridor: start shifted by -3 (outward in negative X), end deeper (+6).
        Pose2d intake2PoseStart = new Pose2d(INTAKE_START_X + INTAKE2_START_OFFSET_X, INTAKE2_Y, Math.toRadians(0));
        Pose2d intake2PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE2_Y, Math.toRadians(0));
        // dodgeGate: -14 in. in X (vs. +14 for red) and -4 in. Y, heading = 40°.
        // The negative X shift steers the robot AWAY from the blue-side gate (at +12 X)
        // before the spline back to the shooting pose.
        Pose2d dodgeGate = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset - 14, INTAKE2_Y - 4, Math.toRadians(40));

        // Row-3 intake corridor: start shifted by -5, same end depth as row 2.
        Pose2d intake3PoseStart = new Pose2d(INTAKE_START_X + INTAKE3_START_OFFSET_X, INTAKE3_Y, Math.toRadians(0));
        Pose2d intake3PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE3_Y, Math.toRadians(0));

        // Parking pose: (-6, 68, 0°). On the negative-X side, facing positive X.
        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(0));

        // toShoot: Drive from start to the blue shooting pose. Initializes indexer to slot 2
        // and spins flywheel simultaneously, then fires after 1.65 s.
        Action toShoot = new ParallelAction(
                // Single-segment spline-heading trajectory from (0,0,0°) to (-17, 40, -50°).
                drive.actionBuilder(startPose)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading)
                        .build(),

                // Spin flywheel to 3240 RPM during drive.
                botActions.actionStartOuttake(SHOOT_RPM),
                // Position the indexer drum to the pre-loaded slot 2.
                botActions.initializeAuto(Indexer.IndexerState.two),

                new SequentialAction(
                        // Wait 1.65 s for arrival and flywheel stabilization.
                        new SleepAction(timeUntilStartOuttake),
                        // Fire the first pre-loaded artifact.
                        botActions.actionQuickOuttake()
                )
        );

        // intake1: Feedback-driven sweep of row 1 at 17 in/s toward positive X.
        Action intake1 = botActions.actionIntakeThreeFeedback(shootingPose, intake1PoseStart, intake1PoseEnd, drive, maxIntakeDrivingVel);

        // goToGate: Optional route to the blue-side gate before returning to shoot.
        // Currently commented out in the active sequence.
        Action goToGate = drive.actionBuilder(intake1PoseEnd)
                .strafeToLinearHeading(gate.position, gate.heading)
                .waitSeconds(gateWaitTime)
                .build();

        // backToShoot1: Direct return from row-1 end to blue shooting pose. Fires artifact 2.
        Action backToShoot1 = new ParallelAction(
                // Direct spline-heading return from row-1 end (positive X side) to shooting pose.
                drive.actionBuilder(intake1PoseEnd) // goToGate
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Flywheel re-spinup during return drive.
                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        // 1.65 s — short return from row 1.
                        new SleepAction(timeUntilStartOuttake),
                        // Fire second artifact immediately on arrival.
                        botActions.actionQuickOuttake()
                )
        );

        // intake2: Feedback-driven sweep of row 2 (Y ≈ 76.5 in.) at 17 in/s.
        Action intake2 = botActions.actionIntakeThreeFeedback(shootingPose, intake2PoseStart, intake2PoseEnd, drive, maxIntakeDrivingVel);

        // backToShoot2: Two-segment return from row-2 end; avoids the blue-side gate via dodgeGate.
        // dodgeGate shifts -14 in. in X (away from gate at +12 X) then splines to shoot.
        Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        // Arc through dodgeGate (40°) to clear the blue-side gate obstacle.
                        .strafeToSplineHeading(dodgeGate.position, dodgeGate.heading)
                        // Then spline-heading into the blue shooting pose.
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Flywheel spinup in parallel with the two-segment drive.
                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        // +0.5 s extra for the longer two-segment return.
                        new SleepAction(timeUntilStartOuttake + 0.5),
                        // Fire third artifact.
                        botActions.actionQuickOuttake()
                )
        );

        /*Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        .setTangent(Math.toRadians(0))

                        .splineToSplineHeading(
                                shootingPose,
                                Math.toRadians(160)
                        )
                        .build(),

                botActions.actionStartOuttake(SHOOT_RPM),
                botActions.rotateToMotifColorBeforeOuttake(2, botActions::getObeliskId, 0),

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake + 0.5),
                        botActions.actionQuickOuttake()
                )
        );*/

        // intake3: Feedback-driven sweep of row 3 (Y ≈ 96 in.) at 17 in/s.
        Action intake3 = botActions.actionIntakeThreeFeedback(shootingPose, intake3PoseStart, intake3PoseEnd, drive, maxIntakeDrivingVel);

        // backToShoot3: Direct return from row-3 end to the shooting pose.
        // No gate dodge needed — row 3 is far enough from the gate that a direct path clears it.
        Action backToShoot3 = new ParallelAction(
                // Single-segment spline-heading return from row-3 end.
                drive.actionBuilder(intake3PoseEnd)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                // Flywheel spinup in parallel.
                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        // +1.0 s extra for the longest return trip (from row 3).
                        new SleepAction(timeUntilStartOuttake + 1.0),
                        // Fire fourth and final artifact.
                        botActions.actionQuickOuttake()
                )
        );

        // toPark: Final spline from the shooting pose to the blue parking zone.
        Action toPark = drive.actionBuilder(shootingPose)
                .strafeToSplineHeading(
                        parkPose.position,  // Target: (-6, 68)
                        parkPose.heading    // Final heading: 0°
                )
                .build();

        // Wait for the Drive Station "Play" button.
        waitForStart();
        // Exit immediately if emergency-stopped before match start.
        if (isStopRequested()) return;

        // Execute the full routine, blocking until all actions complete.
        Actions.runBlocking(
                new ParallelAction(
                        // Thread 1 — Periodic: telemetry and sensor maintenance.
                        botActions.actionPeriodic(),
                        // Thread 2 — Main sequential mission.
                        new SequentialAction(
                                toShoot,        // Drive to blue shooting pose; init; fire artifact 1
                                intake1,        // Sweep row 1 (Y ≈ 50 in.) toward positive X
                                /*goToGate,*/   // (Disabled) Optional gate approach
                                backToShoot1,   // Return to shoot; fire artifact 2
                                intake2,        // Sweep row 2 (Y ≈ 76.5 in.)
                                backToShoot2,   // Dodge blue-side gate; fire artifact 3
                                intake3,        // Sweep row 3 (Y ≈ 96 in.)
                                backToShoot3,   // Return to shoot; fire artifact 4
                                toPark          // Drive to blue parking zone
                        )
                )
        );
    }
}