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
 * <h1>FasterBlueCloseMotif – Blue Alliance Close-Side Autonomous with Motif Detection</h1>
 *
 * <p>This is the primary high-cycle autonomous OpMode for Team 5273 "Decode" on the
 * <strong>Blue Alliance, close (front) side</strong> of the field. It implements a full
 * three-cycle intake-and-shoot routine combined with Limelight-based Motif (AprilTag /
 * colored-pattern) detection so the robot scores artifacts in the correct scoring positions.</p>
 *
 * <h2>High-Level Autonomous Sequence</h2>
 * <ol>
 *   <li><b>Initialization</b> – Hardware is set up; the indexer is pre-loaded to slot two;
 *       the flywheel spins up to {@link #SHOOT_RPM} RPM in parallel with driving.</li>
 *   <li><b>Drive to Obelisk &amp; Shoot (#1)</b> – The robot strafes through the obelisk
 *       scan position so the Limelight camera can detect the Motif ID, then drives to the
 *       shooting pose. While travelling, the indexer rotates to the correct color slot,
 *       and a quick outtake fires the first artifact.</li>
 *   <li><b>Intake Cycle 1</b> – The robot drives to the first intake row (INTAKE1_Y),
 *       uses sensor feedback to confirm three artifacts are loaded, then returns to the
 *       shooting pose and fires again (second shot).</li>
 *   <li><b>Intake Cycle 2</b> – Repeats the intake process at a deeper row (INTAKE2_Y).
 *       The return path includes a dodge waypoint to avoid the gate obstacle before
 *       returning to the shooting pose for the third shot.</li>
 *   <li><b>Intake Cycle 3</b> – Deepest intake row (INTAKE3_Y), followed by a fourth
 *       shot and final intake-passive state.</li>
 *   <li><b>Park</b> – Robot drives to the designated parking zone.</li>
 * </ol>
 *
 * <h2>Parallelism Strategy</h2>
 * <p>Road Runner's {@link ParallelAction} is used extensively so that the flywheel
 * spin-up, indexer rotation, and drivetrain movement all happen simultaneously, saving
 * several seconds of cycle time. The {@link #timeUntilStartOuttake} constant is the
 * critical tuning value that determines when the outtake sequence begins relative to the
 * start of the drive segment – too early and the robot fires before reaching the goal;
 * too late and time is wasted waiting.</p>
 *
 * <h2>Tunable Parameters (FTC Dashboard / @Config)</h2>
 * <p>All {@code public static} fields are exposed to FTC Dashboard via the {@link Config}
 * annotation, enabling real-time tuning without recompiling:</p>
 * <ul>
 *   <li>{@link #maxIntakeDrivingVel} – caps robot speed while intaking to improve pickup reliability.</li>
 *   <li>{@link #OBELISK_X}, {@link #OBELISK_Y}, {@link #OBELISK_HEADING_DEG} – scan waypoint.</li>
 *   <li>{@link #SHOOT_X}, {@link #SHOOT_Y}, {@link #SHOOT_HEADING_DEG} – shooting pose.</li>
 *   <li>{@link #SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT} – small heading correction applied on
 *       subsequent shots to account for drift after the first cycle.</li>
 *   <li>{@link #INTAKE_START_X} / {@link #INTAKE_END_X} and related Y values – define the
 *       start and end X positions for each intake sweep row.</li>
 *   <li>{@link #gate_X}, {@link #gate_Y}, {@link #gateWaitTime} – gate interaction point
 *       (currently bypassed in the active path).</li>
 *   <li>{@link #PARK_X}, {@link #PARK_Y} – final parking position.</li>
 *   <li>{@link #SHOOT_RPM} – target flywheel RPM for consistent shooting velocity.</li>
 *   <li>{@link #timeUntilStartOuttake} – delay (seconds) from the beginning of each
 *       drive-to-shoot segment until the outtake sequence fires.</li>
 * </ul>
 *
 * <h2>Key Dependencies</h2>
 * <ul>
 *   <li>{@link Hardware} – initialises all motors, servos, sensors, and subsystems.</li>
 *   <li>{@link BotActions} – factory for all composite robot actions (intake, outtake, indexer rotate, etc.).</li>
 *   <li>{@link MecanumDrive} – Road Runner mecanum drive with trajectory following.</li>
 *   <li>{@link Indexer} – three-slot drum that stores artifacts; pre-set to slot two at start.</li>
 * </ul>
 *
 * @see FarLeave
 * @see TestingOpmode
 */
@Config
@Autonomous(name = "Faster Blue Auto With Motif", group = "Autonomous")
public class FasterBlueCloseMotif extends LinearOpMode {

    /** Maximum translational velocity (inches/sec) allowed while the intake is running.
     *  Slowing down during intake sweeps improves the chance of picking up artifacts. */
    public static double maxIntakeDrivingVel = 15;

    /** X coordinate (inches) of the intermediate waypoint used to pass near the Obelisk
     *  so the Limelight camera can scan the Motif pattern. Negative = toward the alliance wall. */
    public static double OBELISK_X = -19.5;

    /** Y coordinate (inches) of the Obelisk scan waypoint. */
    public static double OBELISK_Y = 13;

    /** Heading (degrees) the robot faces while passing through the Obelisk scan waypoint.
     *  A small negative angle keeps the camera pointed toward the Motif. */
    public static double OBELISK_HEADING_DEG = -15;

    /** X coordinate (inches) of the shooting pose – where the robot stops to fire artifacts. */
    public static double SHOOT_X = -17;

    /** Y coordinate (inches) of the shooting pose. */
    public static double SHOOT_Y = 40;

    /** Heading (degrees) of the shooting pose, angled toward the high goal. */
    public static double SHOOT_HEADING_DEG = -45;

    /** Small heading offset (degrees) added to the shooting pose on every cycle after the
     *  first shot. This compensates for accumulated odometry drift so subsequent shots
     *  remain on target. Positive = rotate slightly counter-clockwise. */
    public static double SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT = 2; // 2

    /** Starting X coordinate (inches) for the first intake sweep. The robot begins the
     *  intake motion here (facing 0°) and drives forward to {@link #INTAKE_END_X}. */
    public static double INTAKE_START_X = -12;

    /** Additional X offset (inches) applied to the intake 2 start position so the robot
     *  begins the second sweep slightly further forward, accounting for field geometry. */
    public static double INTAKE2_START_OFFSET_X = 3.0;

    /** Additional X offset (inches) for the intake 3 start position (deepest sweep). */
    public static double INTAKE3_START_OFFSET_X = 5.0;

    /** X coordinate (inches) at which the robot ends each intake sweep for row 1.
     *  The intake should have collected artifacts by the time the robot reaches here. */
    public static double INTAKE_END_X = 18.5;

    /** Extra X distance (inches) added to INTAKE_END_X for intake rows 2 and 3, because
     *  those rows are deeper in the field and require the robot to travel further. */
    public static double intake_END_2And3_XOffset = 7.5;

    /** Y coordinate (inches) of the first intake row – closest to the wall. */
    public static double INTAKE1_Y = 52;

    /** Y coordinate (inches) of the second intake row. */
    public static double INTAKE2_Y = 76;

    /** Y coordinate (inches) of the third (deepest) intake row. */
    public static double INTAKE3_Y = 96;

    /** X coordinate (inches) of the gate/ramp position used in the unused {@code goToGate} path. */
    public static double gate_X = 12;

    /** Y coordinate (inches) of the gate/ramp position. */
    public static double gate_Y = 60;

    /** How many seconds the robot waits at the gate position (used in the currently bypassed
     *  {@code goToGate} action). */
    public static double gateWaitTime = 1.5;

    /** X coordinate (inches) of the final parking pose at the end of autonomous. */
    public static double PARK_X = -6;

    /** Y coordinate (inches) of the final parking pose. */
    public static double PARK_Y = 68;

    /** Target flywheel speed in RPM. 3580 RPM is tuned for consistent shot trajectory
     *  at the shooting pose distance from the goal. */
    public static int SHOOT_RPM = 3580;

    /** Seconds after the start of each drive-to-shoot segment to wait before
     *  triggering the outtake sequence. This value must be slightly less than the
     *  expected drive time so the indexer finishes rotating before the robot arrives.
     *  Includes the internal wait for the actuator/gate to open. */
    public static double timeUntilStartOuttake = 2.75; // Time until you start the outtake action, which still includes the wait for actuator

    // has quick outtake and quick intake
    // bunch of compensations for bad rr
    /**
     * Main autonomous execution method called by the FTC SDK when the OpMode starts.
     *
     * <p>All trajectory construction and action composition happens here before
     * {@code waitForStart()} so that the robot is ready to execute instantly when the
     * match begins. After the start signal the pre-built action tree is handed to
     * {@link Actions#runBlocking} which drives the robot through the entire sequence.</p>
     *
     * <p><b>Threading note:</b> Road Runner actions run on the main loop thread.
     * {@link ParallelAction} interleaves actions by calling each one's {@code run()}
     * method every loop iteration – they are cooperative, not truly parallel threads.</p>
     */
    @Override
    public void runOpMode() {
        // The robot starts facing 180° (toward the opposing alliance wall on the blue side).
        // All coordinates below are relative to this starting pose.
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

        // Initialise all hardware (motors, servos, sensors) using the FTC hardwareMap.
        // The Hardware class also wires up the Road Runner localizer with the start pose.
        Hardware hardware = new Hardware(hardwareMap, telemetry, this, startPose);

        // BotActions provides high-level composite actions (intake, outtake, indexer, etc.)
        BotActions botActions = hardware.actions;

        // MecanumDrive provides trajectory building and following for the mecanum drivetrain.
        MecanumDrive drive = hardware.mecanumDrive;

        // Intermediate scan waypoint: the robot passes close to the Obelisk structure so
        // the Limelight camera captures the Motif (AprilTag-like pattern) and determines
        // which slot colour should be shot first.
        Pose2d obeliskPose = new Pose2d(
                OBELISK_X,
                OBELISK_Y,
                Math.toRadians(OBELISK_HEADING_DEG)
        );

        // The pose where the robot stops (or near-stops) to fire artifacts into the goal.
        // The heading is angled at -45° to aim the flywheel toward the high scoring target.
        Pose2d shootingPose = new Pose2d(
                SHOOT_X,
                SHOOT_Y,
                Math.toRadians(SHOOT_HEADING_DEG)
        );

        // Intake row 1: start and end poses. The robot faces 0° (toward the field centre)
        // during the intake sweep so the intake funnel is in the correct orientation.
        Pose2d intake1PoseStart = new Pose2d(INTAKE_START_X, INTAKE1_Y, Math.toRadians(0));
        Pose2d intake1PoseEnd = new Pose2d(INTAKE_END_X, INTAKE1_Y, Math.toRadians(0));

        // Gate pose (90° heading = robot facing sideways toward the gate mechanism).
        // Used in the currently bypassed goToGate action.
        Pose2d gate = new Pose2d(gate_X, gate_Y, Math.toRadians(90));

        // Intake row 2 poses. The start X is shifted forward by INTAKE2_START_OFFSET_X
        // because row 2 artifacts are positioned deeper in the field.
        Pose2d intake2PoseStart = new Pose2d(INTAKE_START_X + INTAKE2_START_OFFSET_X, INTAKE2_Y, Math.toRadians(0));
        Pose2d intake2PoseEnd = new Pose2d(INTAKE_END_X + intake_END_2And3_XOffset, INTAKE2_Y, Math.toRadians(0));

        // Intermediate waypoint used in backToShoot2 to steer around the physical gate
        // obstacle that sits between the intake end and the shooting pose. The robot
        // first drives to this dodge point before curving toward the shooting pose.
        Pose2d dodgeGate = new Pose2d(INTAKE_END_X + intake_END_2And3_XOffset - 16, INTAKE2_Y + 4, Math.toRadians(20));

        // Intake row 3 poses – the deepest intake position on the field.
        Pose2d intake3PoseStart = new Pose2d(INTAKE_START_X + INTAKE3_START_OFFSET_X, INTAKE3_Y, Math.toRadians(0));
        Pose2d intake3PoseEnd = new Pose2d(INTAKE_END_X + intake_END_2And3_XOffset, INTAKE3_Y, Math.toRadians(0));

        // Final park position after all shooting cycles are complete.
        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(0));

        // Remove actionStartOuttake in toShoot when adding this
        // Startup parallel action: initialise the indexer to slot two (pre-loaded artifact)
        // while simultaneously spinning the flywheel up to shoot RPM, saving ~1 second.
        Action start = new ParallelAction(
                botActions.initializeAuto(Indexer.IndexerState.two),  // rotate indexer drum to slot 2
                botActions.actionStartOuttake(SHOOT_RPM)              // spin flywheel to target RPM
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

        // BE CAREFUL ABOUT THE OBELISK SCANNING AND COLOR ROTATING TIMING
        // Drive from start → obelisk scan waypoint → shooting pose, while simultaneously:
        //   • spinning up the flywheel
        //   • waiting for the obelisk ID to be detected, then rotating the indexer to the
        //     correct colour slot, and finally firing the first quick outtake
        // The 0.5-second lead time on the sleep ensures the indexer finishes rotating
        // before the robot fully arrives at the shooting pose.
        Action toShoot = new ParallelAction(
                drive.actionBuilder(startPose) // obeliskPose
                        .strafeTo(obeliskPose.position)                                  // pass near obelisk for camera scan
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading) // curve into shooting pose
                        .build(),

                botActions.actionStartOuttake(SHOOT_RPM), // spin flywheel in parallel with driving

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake - 0.5),                          // wait until robot is nearly at goal
                        botActions.rotateToMotifColorBeforeOuttake(0, botActions::getObeliskId, 2), // rotate indexer slot 0 to Motif-detected colour
                        new SleepAction(0.5),                                                  // brief pause to let indexer settle
                        botActions.actionQuickOuttake()                                        // fire first artifact
                )
        );

        // First intake sweep: uses sensor feedback (actionIntakeThreeFeedback) to drive
        // from the shooting pose into row 1 and back-detect when three artifacts are loaded.
        Action intake1 = botActions.actionIntakeThreeFeedback(shootingPose, intake1PoseStart, intake1PoseEnd, drive, maxIntakeDrivingVel);

        // Build a trajectory to the gate position and wait there (bypassed in the active run).
        // Kept for reference / future use if the gate strategy is re-enabled.
        Action goToGate = drive.actionBuilder(intake1PoseEnd)
                .strafeToLinearHeading(gate.position, gate.heading) // drive to gate with final linear heading
                .waitSeconds(gateWaitTime)                          // wait at gate (e.g. for gate to open)
                .build();

        // Return from intake row 1 to shooting pose. The heading is slightly offset from
        // the base shooting heading to compensate for drift accumulated during intake cycle 1.
        // Parallel: spin up flywheel, then after delay rotate indexer slot 1 to correct
        // colour, wait for robot to arrive, fire second artifact, set intake to passive.
        Action backToShoot1 = new ParallelAction(
                drive.actionBuilder(intake1PoseEnd) // goToGate
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                //botActions.actionSetIntakeReverse(),
                botActions.actionStartOuttake(SHOOT_RPM), // re-spin flywheel (may have slowed during intake)

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake - 1),                                // wait for robot to near shooting pose
                        botActions.rotateToMotifColorBeforeOuttake(1, botActions::getObeliskId, 2),// rotate indexer slot 1 to correct colour
                        new SleepAction(1),                                                        // wait for indexer to settle
                        botActions.actionQuickOuttake(),                                           // fire second artifact
                        botActions.actionSetIntakePassive()                                        // set intake to passive (no active spin) ready for next cycle
                )
        );

        // Second intake sweep: row 2 (deeper in field). Uses the same sensor-feedback
        // approach; intake start/end X offsets account for the deeper artifact positions.
        Action intake2 = botActions.actionIntakeThreeFeedback(shootingPose, intake2PoseStart, intake2PoseEnd, drive, maxIntakeDrivingVel);

        // Return from intake row 2 to shooting pose, routing through dodgeGate waypoint
        // to avoid the physical gate obstacle. Uses strafeTo for the dodge, then
        // strafeToSplineHeading for a smooth curve into the shooting heading.
        Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        .strafeTo(dodgeGate.position)                                            // drive around gate obstacle
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                //botActions.actionSetIntakeReverse(),
                botActions.actionStartOuttake(SHOOT_RPM), // spin up flywheel for third shot

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake - 1),                                // wait until nearly at shooting pose
                        botActions.rotateToMotifColorBeforeOuttake(1, botActions::getObeliskId, 2),// rotate indexer slot 1 to correct colour
                        new SleepAction(1),                                                        // settle pause
                        botActions.actionQuickOuttake(),                                           // fire third artifact
                        botActions.actionSetIntakePassive()                                        // reset intake to passive
                )
        );

        // Alternative spline-based return from intake row 2 (not used in the active run –
        // kept as a reference/fallback). Uses splineTo for a smoother arc around the dodge
        // waypoint and splineToSplineHeading for a tangent-continuous arrival at shooting pose.
        Action backToShoot2Spline = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        //.setTangent()
                        .splineTo(dodgeGate.position, dodgeGate.heading)  // smooth spline arc around gate
                        .splineToSplineHeading(
                                new Pose2d(
                                        shootingPose.position,
                                        shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT))
                                ),
                                shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT))
                        )
                        .build(),

                //botActions.actionSetIntakeReverse(),
                botActions.actionStartOuttake(SHOOT_RPM), // spin up flywheel

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake - 1),
                        botActions.rotateToMotifColorBeforeOuttake(2, botActions::getObeliskId, 2), // slot 2 for third cycle
                        new SleepAction(1),
                        botActions.actionQuickOuttake(),
                        botActions.actionSetIntakePassive()
                )
        );

        // Third intake sweep at the deepest row. Deepest X start offset applied.
        Action intake3 = botActions.actionIntakeThreeFeedback(shootingPose, intake3PoseStart, intake3PoseEnd, drive, maxIntakeDrivingVel);

        // Return from intake row 3 to shooting pose for the fourth and final shot.
        // No dodge waypoint needed from row 3 (different field geometry).
        Action backToShoot3 = new ParallelAction(
                drive.actionBuilder(intake3PoseEnd)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                //botActions.actionSetIntakeReverse(),
                botActions.actionStartOuttake(SHOOT_RPM), // spin up flywheel for final shot

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake - 1),
                        botActions.rotateToMotifColorBeforeOuttake(3, botActions::getObeliskId, 2), // slot 3 for final cycle
                        new SleepAction(1),
                        botActions.actionQuickOuttake(),      // fire final artifact
                        botActions.actionSetIntakePassive()   // disable intake after last cycle
                )
        );

        // Drive from the shooting pose to the parking zone using a spline heading transition
        // so the robot arrives facing the correct direction for the end-game.
        Action toPark = drive.actionBuilder(shootingPose)
                .strafeToSplineHeading(
                        parkPose.position,
                        parkPose.heading
                )
                .build();

        // Block until the driver presses START on the Driver Station.
        waitForStart();

        // Immediately exit if the stop button was pressed instead of start (e.g. e-stop).
        if (isStopRequested()) return;

        // Execute the entire autonomous routine. Actions.runBlocking loops until every
        // action in the tree returns false (i.e. finished).
        Actions.runBlocking(
                new ParallelAction(
                        // Lane 1: After a short 0.5-second delay (to let the robot move away from
                        // the start wall and get a clear camera view), trigger the Limelight scan
                        // of the Obelisk to identify the Motif colour pattern.
                        new SequentialAction(
                                new SleepAction(0.5),           // wait for robot to clear the starting wall
                                botActions.actionScanObelisk()  // capture Motif ID from Limelight
                        ),

                        // Lane 2: Run the periodic hardware update loop (telemetry, sensor polling)
                        // throughout the entire autonomous period.
                        botActions.actionPeriodic(),

                        // Lane 3: Main robot drive sequence executed in order.
                        new SequentialAction(
                                new InstantAction(() -> drive.localizer.setPose(startPose)), // reset odometry to known start pose
                                start,          // initialise indexer + spin up flywheel
                                //toObelisk,    // (disabled) original separate obelisk-scan drive action
                                toShoot,        // drive through obelisk → shoot first artifact
                                intake1,        // sweep row 1 intake
                                backToShoot1,   // return → shoot second artifact
                                intake2,        // sweep row 2 intake
                                backToShoot2,   // return (via gate dodge) → shoot third artifact
                                intake3,        // sweep row 3 intake (deepest)
                                backToShoot3,   // return → shoot fourth artifact
                                toPark          // drive to parking zone
                        )
                )
        );
    }
}