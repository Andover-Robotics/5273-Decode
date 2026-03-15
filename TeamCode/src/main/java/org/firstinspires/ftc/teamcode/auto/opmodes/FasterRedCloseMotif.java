package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

/**
 * FasterRedCloseMotif — Red Alliance, Close-Side Autonomous with Motif (Obelisk) Scanning
 *
 * <p><b>Alliance:</b> Red &nbsp;|&nbsp; <b>Starting Position:</b> Close side (robot begins near
 * the red driver's station, facing the field interior at 180°).</p>
 *
 * <h2>Mission Overview</h2>
 * <p>This OpMode executes a fully automated 30-second autonomous routine:
 * <ol>
 *   <li>Initializes the indexer drum and spins up the flywheel to {@link #SHOOT_RPM} RPM.</li>
 *   <li>Drives to the shooting position via a two-step splined trajectory, while the
 *       Limelight camera reads the Obelisk AprilTag (the "Motif") in the background.
 *       The tag encodes which color artifact (GREEN or PURPLE) belongs at each scoring
 *       structure on the field.</li>
 *   <li>Rotates the 3-slot indexer drum ({@code rotateToMotifColorBeforeOuttake}) to align
 *       the correct-colored artifact with the shooter, then fires the first preloaded artifact.</li>
 *   <li>Performs three intake cycles at Y ≈ 52, 76, and 96 inches from start, collecting
 *       field artifacts and returning to the shooting pose to fire a color-matched artifact
 *       after each cycle.</li>
 *   <li>Parks in the designated zone for the parking bonus.</li>
 * </ol>
 * </p>
 *
 * <h2>"Motif" Variant vs. No-Motif</h2>
 * <p>The "Motif" variants call {@code rotateToMotifColorBeforeOuttake} before every shot.
 * The Limelight camera continuously decodes the Obelisk AprilTag ID ({@code getObeliskId}),
 * which tells the robot which color is required at each scoring slot. The indexer drum
 * then rotates to present that color before the actuator gate opens. This scores the right
 * color automatically rather than relying on manual pre-loading arrangement.</p>
 *
 * <h2>Key Tuning Differences from {@link FasterRedClose} (No-Motif)</h2>
 * <ul>
 *   <li>Uses {@code rotateToMotifColorBeforeOuttake} (category 2 = wait for confirmation)
 *       before each of the four shots (slots 0–3).</li>
 *   <li>Higher {@link #SHOOT_RPM} (3580 vs. 3240) to maintain shot power regardless of
 *       which indexer drum slot ends up aligned with the shooter.</li>
 *   <li>Slower {@link #maxIntakeDrivingVel} (15 in/s vs. 30) for more reliable artifact
 *       capture — slower drive speed gives the intake rollers more contact time.</li>
 *   <li>Two-step spline {@code toShoot} trajectory (via an intermediate waypoint 9 in. short
 *       and 14 in. below the final position) to create a smoother heading approach arc.</li>
 *   <li>An alternative {@code backToShoot2Spline} (using full cubic splines) is kept as a
 *       fallback for the row-2 return, but the active path uses {@code backToShoot2}
 *       (strafeTo dodgeGate + strafeToSplineHeading) for better gate avoidance reliability.</li>
 * </ul>
 *
 * <h2>Coordinate System</h2>
 * <p>All positions are in inches relative to the robot's start pose (0, 0, 180°).
 * Positive Y runs away from the starting wall toward the far side of the field.
 * Positive X runs toward the red alliance wall (rightward when facing from the red side).
 * Headings are measured counter-clockwise from positive X; the robot starts at 180°
 * (facing toward the field interior / negative-X direction).</p>
 *
 * <h2>FTC Dashboard Tuning</h2>
 * <p>{@code @Config} exposes every {@code public static} field to the FTC Dashboard web
 * interface, enabling live position, heading, RPM, and timing adjustments without
 * recompiling the code.</p>
 *
 * @see FasterRedClose        Non-motif variant for the red close side
 * @see FasterBlueCloseMotif  Blue alliance mirror of this autonomous
 * @see BotActions            Factory for all subsystem Road Runner actions
 * @see Hardware              Initializes drivetrain, indexer, intake, shooter, and camera
 */
@Config
@Autonomous(name = "Faster Red Auto With Motif", group = "Autonomous")
public class FasterRedCloseMotif extends LinearOpMode {

    /**
     * Maximum translational velocity (inches per second) while the robot drives through the
     * intake corridors. Set conservatively at 15 in/s (vs. 30 for non-motif) so the intake
     * rollers have longer contact time with each artifact, reducing missed pickups and
     * minimizing odometry slip that could corrupt the pose estimate.
     */
    public static double maxIntakeDrivingVel = 15;

    /**
     * X-coordinate (inches) of the Obelisk (Motif) AprilTag structure.
     * The obelisk is the field element whose AprilTag encodes the color pattern the robot
     * must score. Although the explicit drive-to-obelisk step is commented out in this
     * variant, the pose is preserved for reference and potential re-enablement.
     */
    public static double OBELISK_X = 8;

    /**
     * Y-coordinate (inches) of the Obelisk (Motif) structure.
     * See {@link #OBELISK_X} for full context.
     */
    public static double OBELISK_Y = 36;

    /**
     * Heading (degrees) the robot should face while viewing the Obelisk AprilTag.
     * At -60° the robot has rotated clockwise so its Limelight camera has an optimal
     * sightline to the tag mounted on the obelisk structure.
     */
    public static double OBELISK_HEADING_DEG = -60;

    /**
     * X-coordinate (inches) of the primary shooting position.
     * Chosen to give a clear firing lane toward all scoring targets while remaining
     * clear of the intake corridors and gate obstacle.
     */
    public static double SHOOT_X = 17;

    /**
     * Y-coordinate (inches) of the primary shooting position.
     * At Y = 40 the robot is roughly mid-field on the red alliance side with a clean
     * line of sight to the scoring structures.
     */
    public static double SHOOT_Y = 40;

    /**
     * Heading (degrees) of the robot at the shooting position.
     * At -135° the robot is turned far enough clockwise that the shooter (and the
     * co-located Limelight camera) can see both the scoring targets and the Obelisk
     * AprilTag simultaneously, making color decisions at shot time.
     */
    public static double SHOOT_HEADING_DEG = -135;

    /**
     * Small additional heading offset (degrees) applied to {@link #SHOOT_HEADING_DEG} for
     * all shots after the first. The +2° counter-clockwise nudge compensates for accumulated
     * localization drift between shots and fine-tunes aim after the robot has moved through
     * multiple intake cycles. The inline comment "// -2" marks the prior value that was
     * changed, preserved for tuning history context.
     */
    public static double SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT = 2; // -2

    /**
     * X-coordinate (inches) of the intake sweep start position for all rows.
     * The robot begins its intake sweep from this X value and drives toward negative X
     * (toward the field wall on the red side), sweeping artifacts into the intake rollers.
     */
    public static double INTAKE_START_X = 12;

    /**
     * Inward X offset (inches) subtracted from {@link #INTAKE_START_X} for the second
     * intake row. Shifting the start point inward by 3 in. avoids re-collecting artifacts
     * from row 1 and ensures better alignment with the row-2 artifact positions.
     */
    public static double INTAKE2_START_OFFSET_X = 3.0;

    /**
     * Inward X offset (inches) subtracted from {@link #INTAKE_START_X} for the third
     * intake row. The larger 5 in. shift accounts for the deeper field position of row-3
     * artifacts relative to the robot's normal start-of-sweep location.
     */
    public static double INTAKE3_START_OFFSET_X = 5.0;

    /**
     * X-coordinate (inches) at the end of the first intake sweep.
     * The robot drives from {@link #INTAKE_START_X} to this negative value, sweeping
     * across the full width of the intake corridor to collect all available row-1 artifacts.
     * The negative value places the robot near the far field wall on the red side.
     */
    public static double INTAKE_END_X = -16.5;

    /**
     * Additional X distance (inches) subtracted from {@link #INTAKE_END_X} for the second
     * and third intake rows. Artifacts in rows 2 and 3 are positioned further into the field
     * (more negative X), so the robot must travel deeper to collect them all.
     */
    public static double intake_END_2And3_XOffset = 8;

    /**
     * Y-coordinate (inches) of the first intake row of field artifacts.
     * At ~52 inches from start, this is the nearest row to the robot's launch position,
     * making it the fastest to reach after the first shot.
     */
    public static double INTAKE1_Y = 52;

    /**
     * Y-coordinate (inches) of the second intake row of field artifacts.
     * At ~76 inches, this is approximately mid-field. The return path from here requires
     * navigating around the gate obstacle via the {@code dodgeGate} waypoint.
     */
    public static double INTAKE2_Y = 76;

    /**
     * Y-coordinate (inches) of the third intake row of field artifacts.
     * At ~96 inches, this is the farthest row from the start — near the far end of the field.
     * Despite the distance, the return path is simpler because the robot bypasses the gate.
     */
    public static double INTAKE3_Y = 96;

    /**
     * X-coordinate (inches) of the field gate obstacle.
     * The gate is a physical field element situated between the intake rows and the shooting
     * position. It is referenced when computing the {@code dodgeGate} intermediate waypoint
     * so the robot steers around it on the return from rows 2 and 3.
     */
    public static double gate_X = -12;

    /**
     * Y-coordinate (inches) of the field gate obstacle.
     * At ~60 inches the gate sits roughly between rows 1 and 2, squarely in the most direct
     * return path from rows 2 and 3 back to the shooting position.
     */
    public static double gate_Y = 60;

    /**
     * X-coordinate (inches) of the final parking position.
     * After all four shots are fired, the robot drives here to earn the autonomous parking
     * bonus points.
     */
    public static double PARK_X = 6;

    /**
     * Y-coordinate (inches) of the final parking position.
     * At Y = 68, the parking zone lies between the row-1 intake corridor and the gate obstacle.
     */
    public static double PARK_Y = 68;

    /**
     * Target flywheel speed in RPM for the shooter motor.
     * 3580 RPM provides sufficient projectile velocity from the shooting position to reach
     * the scoring targets. This is higher than the no-motif variant (3240) to maintain
     * consistent shot power regardless of which drum slot is aligned — different drum
     * positions slightly alter the loading geometry and thus the effective launch angle.
     */
    public static int SHOOT_RPM = 3580;

    /**
     * Time in seconds to wait after a parallel drive+spinup action begins before triggering
     * the outtake (firing) sequence. This delay serves three purposes:
     * <ul>
     *   <li>Allows the flywheel to accelerate from rest to {@link #SHOOT_RPM}.</li>
     *   <li>Gives the robot time to settle mechanically into the shooting pose.</li>
     *   <li>Provides a window for the indexer drum color-rotation to complete before
     *       the actuator gate opens.</li>
     * </ul>
     * The actual sleep before firing varies per shot:
     * <ul>
     *   <li>First shot ({@code toShoot}): sleeps {@code timeUntilStartOuttake - 0.5} = 2.0 s,
     *       then rotates, then sleeps 0.5 s more.</li>
     *   <li>Subsequent shots: sleeps {@code timeUntilStartOuttake - 1} = 1.5 s,
     *       then rotates, then sleeps 1 s more.</li>
     * </ul>
     */
    public static double timeUntilStartOuttake = 2.5; // Time until you start the outtake action, which still includes the wait for actuator

    // bunch of compensations for bad rr
    // has quick outtake and quick intake

    /**
     * Main autonomous routine. All Road Runner {@link Action} objects (trajectories and
     * subsystem command sequences) are constructed up front before {@code waitForStart},
     * pre-compiling the spline math so there is no computation delay once the match begins.
     * After start, {@code Actions.runBlocking} executes the entire choreography.
     *
     * <p><b>Full Execution Order:</b>
     * <ol>
     *   <li>{@code start} — Set indexer to slot "two" and spin flywheel to target RPM.</li>
     *   <li>{@code toShoot} — Two-step spline to shooting pose; rotate indexer for slot 0;
     *       fire the first preloaded artifact.</li>
     *   <li>{@code intake1} — Sweep row 1 (Y ≈ 52 in.) collecting artifacts.</li>
     *   <li>{@code backToShoot1} — Return to shooting pose; rotate indexer for slot 1;
     *       fire second artifact.</li>
     *   <li>{@code intake2} — Sweep row 2 (Y ≈ 76 in.) collecting artifacts.</li>
     *   <li>{@code backToShoot2} — Navigate around gate via dodgeGate waypoint; rotate
     *       indexer for slot 2; fire third artifact.</li>
     *   <li>{@code intake3} — Sweep row 3 (Y ≈ 96 in.) collecting artifacts.</li>
     *   <li>{@code backToShoot3} — Return to shooting pose; rotate indexer for slot 3;
     *       fire fourth artifact.</li>
     *   <li>{@code toPark} — Drive to parking zone.</li>
     * </ol>
     * Throughout the entire run, {@code actionScanObelisk} and {@code actionPeriodic} execute
     * as persistent parallel threads, keeping the Limelight scanning and handling background
     * telemetry and sensor updates.</p>
     */
    @Override
    public void runOpMode() {
        // The robot starts at the field origin facing 180° (toward the field interior /
        // negative-X direction). Road Runner uses this seed pose for all downstream
        // trajectory calculations — every position in the route is relative to this origin.
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

        // Initialize all hardware: mecanum drive (with Road Runner two-wheel + IMU localizer),
        // 3-slot indexer drum, intake roller, flywheel shooter, actuator gate, and Limelight.
        // Passing startPose seeds the drive's odometry estimator immediately at construction.
        Hardware hardware = new Hardware(hardwareMap, telemetry, this, startPose);

        // BotActions wraps every subsystem command into a Road Runner Action object so they
        // can be composed with ParallelAction / SequentialAction / SleepAction, making the
        // entire autonomous a single declarative action tree.
        BotActions botActions = hardware.actions;

        // MecanumDrive provides trajectory following via Road Runner's motion profile system.
        // Its actionBuilder API compiles B-spline paths into timed velocity/acceleration profiles.
        MecanumDrive drive = hardware.mecanumDrive;

        // The obelisk pose records where the field's Motif AprilTag structure is located.
        // Even though the robot no longer drives to this pose explicitly (toObelisk is disabled),
        // this value is still used conceptually to understand the tag's field position.
        Pose2d obeliskPose = new Pose2d(
                OBELISK_X,                            // ~8 in. along X
                OBELISK_Y,                            // ~36 in. along Y
                Math.toRadians(OBELISK_HEADING_DEG)   // -60° — angled toward the tag
        );

        // The shooting pose is the robot's position and orientation when firing artifacts.
        // At (17, 40, -135°) the robot is on the near-center area of the red side, rotated
        // so the shooter and Limelight camera both point toward the scoring structures.
        Pose2d shootingPose = new Pose2d(
                SHOOT_X,                               // ~17 in. along X
                SHOOT_Y,                               // ~40 in. along Y
                Math.toRadians(SHOOT_HEADING_DEG)      // -135°
        );

        // Row-1 intake corridor: robot sweeps from X=12 to X=-16.5 at Y=52,
        // collecting the nearest row of field artifacts with the intake rollers running.
        // Heading = 180° keeps the intake facing the correct direction throughout.
        Pose2d intake1PoseStart = new Pose2d(INTAKE_START_X, INTAKE1_Y, Math.toRadians(180));
        Pose2d intake1PoseEnd   = new Pose2d(INTAKE_END_X,   INTAKE1_Y, Math.toRadians(180));

        // The gate pose captures the position of the physical gate obstacle at (−12, 60, 90°).
        // While the goToGate action is currently unused (commented out), the gate position
        // directly informs the dodgeGate waypoint below.
        Pose2d gate = new Pose2d(gate_X, gate_Y, Math.toRadians(90));

        // Row-2 intake corridor: shifted 3 in. inward at the start (to avoid re-collecting
        // row-1 artifacts) and 8 in. deeper at the end (row-2 artifacts are further into the field).
        Pose2d intake2PoseStart = new Pose2d(INTAKE_START_X - INTAKE2_START_OFFSET_X, INTAKE2_Y, Math.toRadians(180));
        Pose2d intake2PoseEnd   = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE2_Y, Math.toRadians(180));

        // dodgeGate is an intermediate waypoint on the return trip from rows 2 and 3.
        // By steering +12 in. in X and +4 in. in Y from the row-2 end position (and angling
        // to 160°), the robot curves around the gate obstacle before the final turn toward
        // the shooting pose. Without this waypoint the straight path would collide with the gate.
        Pose2d dodgeGate = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset + 12, INTAKE2_Y + 4, Math.toRadians(160));

        // Row-3 intake corridor: furthest from start (Y ≈ 96 in.), largest start offset (5 in.)
        // to target the deepest artifacts. End X is same depth as row 2.
        Pose2d intake3PoseStart = new Pose2d(INTAKE_START_X - INTAKE3_START_OFFSET_X, INTAKE3_Y, Math.toRadians(180));
        Pose2d intake3PoseEnd   = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE3_Y, Math.toRadians(180));

        // Parking pose: the robot ends here after firing all four artifacts to claim
        // the autonomous parking bonus. Heading = 180° matches the field's parking zone orientation.
        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(180));

        // --- PRE-COMPILED ACTION DEFINITIONS ---
        // All actions are constructed here before waitForStart so that spline math is resolved
        // and hardware references are bound before the match timer begins.

        // start: Runs before any driving — sets the indexer drum to position "two" (the slot
        // loaded with the first artifact to fire) and starts the flywheel spinning toward
        // target RPM so it reaches full speed by the time the robot arrives at the shooting pose.
        // Note: toObelisk is commented out; obelisk scanning is handled by the persistent outer thread.
        // Remove actionStartOuttake in toShoot when adding this
        Action start = new ParallelAction(
                // Rotate the indexer drum to the "two" position (pre-loaded artifact slot).
                botActions.initializeAuto(Indexer.IndexerState.two),
                // Begin spinning the flywheel motor toward SHOOT_RPM immediately.
                botActions.actionStartOuttake(SHOOT_RPM)
        );

        /*
        // remove soon
        Action toObelisk = new ParallelAction(
                drive.actionBuilder(startPose)
                        .strafeToSplineHeading(obeliskPose.position, obeliskPose.heading)
                        .build(),

                botActions.initializeAuto(Indexer.IndexerState.two),
                botActions.actionStartOuttake(SHOOT_RPM),
                botActions.actionScanObelisk()
        );
        */

        // toShoot: Drives the robot from start to the shooting position while simultaneously
        // spinning the flywheel and timing the indexer rotation + first shot.
        // The two-step spline trajectory uses an intermediate waypoint (SHOOT_X-9, SHOOT_Y-14)
        // to create a smooth curved approach into the final -135° heading — a direct one-step
        // path would produce a sharper, less reliable turn near the shooting pose.
        Action toShoot = new ParallelAction(
                // Two-segment spline path: first arc to intermediate point, then curve to shooting pose.
                // The intermediate point (8 in. short, 14 in. behind) shapes a smooth entry arc.
                drive.actionBuilder(startPose) // obeliskPose
                        .strafeToSplineHeading(new Vector2d(shootingPose.position.x - 9, shootingPose.position.y - 14), SHOOT_HEADING_DEG)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading)
                        .build(),

                // Maintain flywheel at target RPM throughout the drive (runs concurrently).
                botActions.actionStartOuttake(SHOOT_RPM),

                // Sequential outtake preparation — timed to execute as the robot settles at the pose:
                new SequentialAction(
                        // Sleep 2.0 s (= timeUntilStartOuttake - 0.5) to let the robot reach the
                        // shooting pose and the flywheel stabilize near target RPM.
                        new SleepAction(timeUntilStartOuttake - 0.5),
                        // Rotate the indexer drum to the color required for scoring slot 0.
                        // getObeliskId supplies the AprilTag ID decoded by the Limelight camera.
                        // Category 2 means the action will wait for the drum to confirm its position.
                        botActions.rotateToMotifColorBeforeOuttake(0, botActions::getObeliskId, 2),
                        // 0.5 s settle time after drum rotation before the gate opens —
                        // ensures the drum is fully seated before the artifact is launched.
                        new SleepAction(0.5),
                        // Open the actuator gate; the spinning flywheel propels the artifact toward
                        // the scoring target. "Quick" outtake minimizes the open time to avoid
                        // accidentally ejecting the next artifact in the drum.
                        botActions.actionQuickOuttake()
                )
        );

        // intake1: Full feedback-driven intake sequence for row 1 (Y ≈ 52 in.).
        // actionIntakeThreeFeedback drives the robot along the intake corridor at maxIntakeDrivingVel
        // (15 in/s) while running the intake rollers. It uses sensor feedback (likely color sensor
        // or motor current) to detect when each of the three indexer drum slots has been filled,
        // stopping or reversing as needed to ensure all three are loaded before returning.
        Action intake1 = botActions.actionIntakeThreeFeedback(shootingPose, intake1PoseStart, intake1PoseEnd, drive, maxIntakeDrivingVel);


        // backToShoot1: Returns from the row-1 end position to the shooting pose and fires
        // the second artifact (indexer slot 1). Row 1 is close enough that a single-segment
        // spline suffices — no gate-avoidance waypoint is needed.
        Action backToShoot1 = new ParallelAction(
                // Direct spline-heading path from row-1 end to the shooting pose.
                // The +2° heading offset (SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT) compensates
                // for any slight drift that accumulated during the first intake cycle.
                drive.actionBuilder(intake1PoseEnd) // goToGate
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                //botActions.actionSetIntakeReverse(),
                // Spin the flywheel up in parallel with the drive so it is ready on arrival.
                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        // Sleep 1.5 s (= timeUntilStartOuttake - 1) — shorter than the first shot
                        // because this return trip is brief and the flywheel stays partially spun up.
                        new SleepAction(timeUntilStartOuttake - 1),
                        // Rotate drum to the color required for scoring slot 1.
                        botActions.rotateToMotifColorBeforeOuttake(1, botActions::getObeliskId, 2),
                        // 1 s settle time after drum rotation for mechanical stabilization.
                        new SleepAction(1),
                        // Fire the second artifact.
                        botActions.actionQuickOuttake(),
                        // Set intake to passive mode (rollers off, not reversing) to prevent
                        // accidentally ingesting field elements on the next transit.
                        botActions.actionSetIntakePassive()
                )
        );

        // backToShoot2Spline: ALTERNATIVE return trajectory from row 2, kept as a reference.
        // Uses full cubic spline interpolation (splineTo + splineToSplineHeading) instead of
        // the active strafeTo + strafeToSplineHeading approach. Spline paths are smoother and
        // faster but harder to tune reliably around the gate obstacle; the strafeTo variant
        // in backToShoot2 below is the current active choice for consistent gate avoidance.
        // Note: the indexer slot index here is 1 (same as backToShoot1), which appears to be
        // a copy-paste artifact — the active backToShoot2 correctly uses slot index 2.
        Action backToShoot2Spline = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        //.setTangent(Math.toRadians()
                        // Spline through the dodgeGate waypoint (arcing around the gate obstacle).
                        .splineTo(dodgeGate.position, dodgeGate.heading)
                        // Then spline-with-heading interpolation directly into the shooting pose.
                        .splineToSplineHeading(
                                new Pose2d(
                                        shootingPose.position,
                                        shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT))
                                ),
                                shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT))
                        )
                        .build(),

                //botActions.actionSetIntakeReverse(),
                // Flywheel spinup running concurrently with the drive.
                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake - 1),
                        // Note: uses slot 1 — likely a copy-paste error from backToShoot1.
                        botActions.rotateToMotifColorBeforeOuttake(1, botActions::getObeliskId, 2),
                        new SleepAction(1),
                        botActions.actionQuickOuttake(),
                        botActions.actionSetIntakePassive()
                )
        );

        // intake2: Full feedback-driven intake sequence for row 2 (Y ≈ 76 in.).
        // Same mechanism as intake1 but targeting the mid-field artifact positions.
        Action intake2 = botActions.actionIntakeThreeFeedback(shootingPose, intake2PoseStart, intake2PoseEnd, drive, maxIntakeDrivingVel);

        // backToShoot2: Returns from the row-2 end position to the shooting pose, navigating
        // around the gate obstacle, and fires the third artifact (indexer slot 2).
        // Uses a two-segment path: first strafeTo the dodgeGate waypoint (which arcs the robot
        // away from the gate), then strafeToSplineHeading into the shooting pose.
        Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        // Move to dodgeGate first to safely clear the gate obstacle before
                        // heading toward the shooting position.
                        .strafeTo(dodgeGate.position)
                        // Then spline-heading into the final shooting pose with heading offset.
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                //botActions.actionSetIntakeReverse(),
                // Flywheel spinup in parallel with the two-segment drive.
                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        // 1.5 s wait — same as backToShoot1; the two-segment path takes similar time.
                        new SleepAction(timeUntilStartOuttake - 1),
                        // Rotate drum to the color required for scoring slot 2.
                        botActions.rotateToMotifColorBeforeOuttake(2, botActions::getObeliskId, 2),
                        new SleepAction(1),
                        // Fire the third artifact.
                        botActions.actionQuickOuttake(),
                        botActions.actionSetIntakePassive()
                )
        );

        // intake3: Full feedback-driven intake sequence for row 3 (Y ≈ 96 in.), the farthest
        // intake position. Same feedback mechanism as previous rows.
        Action intake3 = botActions.actionIntakeThreeFeedback(shootingPose, intake3PoseStart, intake3PoseEnd, drive, maxIntakeDrivingVel);

        // backToShoot3: Returns from the row-3 end position to the shooting pose and fires
        // the fourth and final artifact (indexer slot 3). Row 3's Y position (96 in.) is far
        // enough from the gate that a direct single-segment spline return is unobstructed —
        // no dodgeGate waypoint is needed here.
        Action backToShoot3 = new ParallelAction(
                // Single-segment spline-heading return from row-3 end to shooting pose.
                drive.actionBuilder(intake3PoseEnd)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                //botActions.actionSetIntakeReverse(),
                // Flywheel spinup in parallel.
                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake - 1),
                        // Rotate drum to the color required for scoring slot 3 (final shot).
                        botActions.rotateToMotifColorBeforeOuttake(3, botActions::getObeliskId, 2),
                        new SleepAction(1),
                        // Fire the fourth and final artifact.
                        botActions.actionQuickOuttake(),
                        botActions.actionSetIntakePassive()
                )
        );

        // toPark: Terminal spline from the shooting pose to the parking zone.
        // The robot ends the autonomous period here to secure the parking bonus points.
        // Heading interpolates from the current shooting angle back to 180° (field-interior facing).
        Action toPark = drive.actionBuilder(shootingPose)
                .strafeToSplineHeading(
                        parkPose.position,  // Drive to (6, 68) — the parking zone coordinates
                        parkPose.heading    // Re-orient to 180° in the parking zone
                )
                .build();

        // Block here until the Drive Station operator presses the Play (▶) button.
        waitForStart();
        // If the OpMode was stopped before it could run (emergency stop before match start),
        // exit immediately without executing any robot motion.
        if (isStopRequested()) return;

        // Execute the entire autonomous choreography. runBlocking keeps the OpMode thread
        // alive until every action in the tree finishes or the OpMode is externally stopped.
        Actions.runBlocking(
                // Outer ParallelAction launches three concurrent execution threads:
                new ParallelAction(
                        // Thread 1 — Obelisk scanner: continuously reads the AprilTag on the
                        // Obelisk via the Limelight camera. Running this as a persistent parallel
                        // thread (rather than a one-shot action) ensures the latest tag ID is always
                        // available when rotateToMotifColorBeforeOuttake polls getObeliskId.
                        botActions.actionScanObelisk(),

                        // Thread 2 — Periodic maintenance: updates telemetry, refreshes sensors,
                        // and keeps any subsystem state machines ticking every loop iteration.
                        botActions.actionPeriodic(),

                        // Thread 3 — Main sequential routine: the robot's step-by-step mission.
                        new SequentialAction(
                                // Instantly re-seed the localizer with the known start pose.
                                // This zeroes out any drift that occurred between hardware
                                // initialization and the actual match start (e.g., the robot
                                // being pushed slightly during last-second field placement).
                                new InstantAction(() -> drive.localizer.setPose(startPose)),

                                start,          // Set indexer to slot 2, spin flywheel to RPM
                                //toObelisk,    // (Disabled) Explicit drive to obelisk for scan
                                toShoot,        // Two-step spline to shooting pose; fire artifact 1
                                intake1,        // Sweep row 1 (Y ≈ 52 in.); fill indexer slots
                                backToShoot1,   // Return to shoot; fire artifact 2 (slot 1)
                                intake2,        // Sweep row 2 (Y ≈ 76 in.); refill indexer
                                backToShoot2,   // Dodge gate; return to shoot; fire artifact 3 (slot 2)
                                intake3,        // Sweep row 3 (Y ≈ 96 in.); refill indexer
                                backToShoot3,   // Return to shoot; fire artifact 4 (slot 3)
                                toPark          // Drive to parking zone for bonus points
                        )
                )
        );
    }
}