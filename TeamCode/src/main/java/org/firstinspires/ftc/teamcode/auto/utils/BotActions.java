package org.firstinspires.ftc.teamcode.auto.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Aimer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

import java.util.function.IntSupplier;

/**
 * BotActions is the autonomous action factory for the robot.
 *
 * <p>Every method in this class returns a Road Runner {@link Action} that
 * encapsulates one logical autonomous step (intake a set of artifacts, perform
 * an outtake dump, scan the obelisk tag, etc.).  The actions are composed
 * together in individual auto OpModes using
 * {@link SequentialAction} / {@link ParallelAction}.
 *
 * <p>The class is annotated with {@link Config} so all {@code public static}
 * tuning constants (spin times, powers, RPM scales) are editable via FTC
 * Dashboard without re-deploying code.
 *
 * <p>All subsystem references are injected through the {@link Hardware} data
 * class so that the same hardware objects are shared with the OpMode.
 */
@Config
public class BotActions {

    // -----------------------------------------------------------------------
    // Injected dependencies
    // -----------------------------------------------------------------------

    /** The running autonomous OpMode; used to check stop/isActive in loop Actions. */
    private final LinearOpMode opMode;

    /** FTC telemetry used to log debug data during Action execution. */
    private final Telemetry telemetry;

    /** Conveyor/intake roller subsystem reference. */
    private final Intake intake;

    /** Three-slot indexer drum subsystem reference. */
    private final Indexer indexer;

    /** Flywheel shooter subsystem reference. */
    private final Outtake outtake;

    /** Actuator (launch ramp or ball-gate) subsystem reference. */
    private final Actuator actuator;

    /** Limelight AprilTag scanner (public so OpModes can call scan/pipeline directly). */
    public final AprilTag aprilTag;

    /** Localization-based heading aimer for auto heading lock. */
    private final Aimer aprilAimer;

    /** Road Runner drive for trajectory actions and pose estimation. */
    private final MecanumDrive drive;

    // -----------------------------------------------------------------------
    // Tuning constants (Dashboard-editable)
    // -----------------------------------------------------------------------

    /** Duration (seconds) of the full-power indexer blast during the quick outtake.
     *  Long enough to eject all three artifacts; shorter = less dead time. */
    public static double NON_INDEX_SPIN_TIME = 2.0;

    /** Raw motor power applied to the indexer drum during the blast phase.
     *  Higher values push artifacts through faster but may cause jamming. */
    public static double FULL_BLAST_POWER = 0.30;

    /** Time (seconds) after the start of the intake trajectory before the indexer
     *  automatically advances to slot 1 to accept the first ball. */
    public static double ball1TimeDisp = 0.66;

    /** Time (seconds) after the start of the intake trajectory before the indexer
     *  automatically advances to slot 2 to accept the second ball. */
    public static double ball2TimeDisp = 1.10;

    /** Total time (seconds) the robot spends over the intake zone before it starts
     *  the return trajectory toward the shooting position. */
    public static double timeToIntake = 2.50;

    /** Whether to maintain continuous AprilTag heading lock during an action sequence.
     *  When {@code true}, {@link #actionPeriodic()} applies heading corrections. */
    private boolean continuousAprilTagLock;

    /** Minimum time (ms) between successive artifact acquisitions in the feedback
     *  intake action.  Prevents double-counting a single artifact that briefly
     *  trips the sensor twice. */
    public static double cooldownFeedbackIntake = 0;

    /** Scale factor applied to the target RPM during the quick-spin outtake.
     *  Values < 1.0 give slightly lower RPM but allow faster spin-up. */
    public static double quickspinRpmScale = 0.93;

    /** The most recent heading turn correction from the AprilTag aimer.
     *  Held between update cycles so correction is not dropped between frames. */
    private double lastTurnCorrection;

    /** The most recently detected obelisk AprilTag ID.
     *  0 = no detection; 21/22/23 = valid motif tag IDs. */
    private int obeliskId = 0;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Constructs a BotActions by extracting all subsystem references from the
     * provided {@link Hardware} container.
     *
     * @param hardware  the robot hardware container (all subsystems already built)
     * @param telemetry FTC telemetry for debug output during actions
     * @param opMode    the running autonomous LinearOpMode (for stop checks)
     */
    public BotActions(
            Hardware hardware,
            Telemetry telemetry,
            LinearOpMode opMode
    ) {
        // Unpack each subsystem reference from the hardware container.
        this.intake    = hardware.intake;
        this.indexer   = hardware.indexer;
        this.outtake   = hardware.outtake;
        this.actuator  = hardware.actuator;
        this.aprilTag  = hardware.aprilTag;
        this.aprilAimer = hardware.aprilAimer;
        this.drive     = hardware.mecanumDrive;
        this.telemetry = telemetry;
        this.opMode    = opMode;
    }

    // -----------------------------------------------------------------------
    // Outtake actions
    // -----------------------------------------------------------------------

    /**
     * Returns an instantaneous {@link Action} that simultaneously enables the
     * indexer's auto-outtake mode and sets the shooter to the given RPM
     * (scaled by {@link #quickspinRpmScale}).
     *
     * <p>Call this before a trajectory to pre-spin the shooter so it is at
     * speed by the time the robot reaches the shooting position.
     *
     * @param rpm the base target RPM for the shooter flywheel
     * @return a {@link ParallelAction} that starts indexer auto-outtake and shooter spin-up
     */
    public Action actionStartOuttake(double rpm) {
        return new ParallelAction(
            new InstantAction(() -> indexer.setAutoOuttaking(true)),           // enable auto-outtake mode
            new InstantAction(() -> outtake.set(rpm * quickspinRpmScale))      // spin up to scaled RPM
        );
    }

    /**
     * Returns an {@link Action} that performs a full "quick dump":
     * raise the actuator, blast all artifacts out with the indexer, then lower
     * and reset everything for the next intake cycle.
     *
     * <p>Sequence:
     * <ol>
     *   <li>Raise actuator to the "quick" (lower) up position.</li>
     *   <li>Wait 350 ms for the actuator to reach position.</li>
     *   <li>Run the indexer drum at {@link #FULL_BLAST_POWER} for
     *       {@link #NON_INDEX_SPIN_TIME} seconds.</li>
     *   <li>Stop the drum; disable auto-outtake mode.</li>
     *   <li>Move indexer to slot 1 (so the next intake starts from slot 0).</li>
     *   <li>Stop the shooter; lower the actuator.</li>
     *   <li>Reset all slot colour assignments to EMPTY.</li>
     * </ol>
     *
     * @return the quick-dump {@link Action} sequence
     */
    public Action actionQuickOuttake() {
        return new SequentialAction(
                new InstantAction(actuator::upQuick),                              // 1. raise actuator (lower "up" position for quick dump)
                new SleepAction(.35),                                              // 2. wait for actuator to reach position (~350 ms)
                new InstantAction(() -> indexer.setIndexerPower(FULL_BLAST_POWER)),// 3. blast drum at full power
                new SleepAction(NON_INDEX_SPIN_TIME),                             // 4. hold blast for configured duration
                new InstantAction(indexer::stopIndexerPower),                     // 5. stop the drum
                new InstantAction(() -> indexer.setAutoOuttaking(false)),          // 6. disable auto-outtake mode
                new InstantAction(() -> indexer.moveTo(Indexer.IndexerState.one)), // 7. home to slot 1 (next intake enters slot 0)
                new InstantAction(outtake::stop),                                  // 8. stop shooter
                new InstantAction(actuator::down),                                 // 9. lower actuator back to intake position
                new InstantAction(indexer::initializeColors)                       // 10. mark all slots EMPTY
        );
    }

    // -----------------------------------------------------------------------
    // Obelisk motif helpers
    // -----------------------------------------------------------------------

    /**
     * Returns an instantaneous {@link Action} that rotates the indexer drum to
     * the correct starting slot for the motif encoded in the obelisk tag.
     *
     * <p>The action reads the obelisk tag ID lazily (at action-run time, not at
     * method-call time) via the {@code id} {@link IntSupplier} so the ID is
     * available only after {@link #actionScanObelisk()} has completed.
     *
     * <p>Algorithm:
     * <ol>
     *   <li>Read the tag ID from {@code id}; abort if not 21/22/23.</li>
     *   <li>Apply the per-row colour assignments to the indexer (rotated for
     *       {@code startingSlot} so the first intaken ball lands in the right
     *       position).</li>
     *   <li>Convert the obelisk ID to the desired shooting color order.</li>
     *   <li>Search all three indexer slots and rotate to the one where the
     *       first two slots match the desired order.</li>
     * </ol>
     *
     * @param row          field row number (0–3) whose colour pattern is being shot
     * @param id           supplier that returns the current obelisk tag ID at run time
     * @param startingSlot the indexer slot index where the first intaken artifact landed
     * @return an {@link InstantAction} that rotates the drum to the correct firing position
     */
    // helpers at the end of the file
    public Action rotateToMotifColorBeforeOuttake(int row, IntSupplier id, int startingSlot) {
        return new InstantAction(() -> {
            // Read the obelisk ID at action-run time (not when this method was called)
            // so that a prior scanObelisk action has time to populate it.
            int tagId = id.getAsInt();
            if (tagId != 21 && tagId != 22 && tagId != 23) {
                // Not a valid motif tag – log and bail out without moving the drum.
                telemetry.addData("No obelisk Id 21, 22 or 23, id is", tagId);
                return;
            }

            // Apply the per-row colour assignments, rotated for the starting slot.
            applyCurrentColorsFromRow(row, startingSlot);

            // Convert the obelisk ID to the desired artifact firing order.
            Indexer.ArtifactColor[] desiredOrder = getDesiredShootOrder(tagId);

            // Search all three indexer slots for the slot whose first two colours
            // match the desired firing order.
            for (Indexer.IndexerState state : Indexer.IndexerState.values()) {
                telemetry.addData("Started search for index of proper", "color");
                if (matchesOrder(state.index, desiredOrder)) {
                    telemetry.addData("Rotated To Motif", "Color");
                    // Rotate the drum to the matching slot so it fires in the right order.
                    Indexer.IndexerState gotoState = state;
                    indexer.moveTo(gotoState, true); // true = block until rotation complete
                    return; // stop after the first matching slot is found
                }
            }
        });
    }

    /**
     * Returns an {@link Action} that switches the indexer back to intake mode
     * at slot two.  Used to reset indexer state between outtake and intake phases.
     *
     * @return a {@link SequentialAction} that disables auto-outtake, enables intake
     *         at slot two, and rotates the drum to slot two
     */
    public Action actionSetSomeShizzle() {
        return new SequentialAction(
                new InstantAction(() -> indexer.setAutoOuttaking(false)),                     // disable auto-outtake
                new InstantAction(() -> indexer.setIntaking(true, Indexer.IndexerState.two)), // enable intake starting at slot 2
                new InstantAction(() -> indexer.moveTo(Indexer.IndexerState.two, true))       // physically rotate drum to slot 2
                );
    }

    // -----------------------------------------------------------------------
    // Intake command helpers
    // -----------------------------------------------------------------------

    /**
     * Returns an instantaneous {@link Action} that runs the intake in slow
     * reverse (used to push partially-captured artifacts back out during reset).
     *
     * @return an {@link InstantAction} that calls {@code intake.runBackwardsSlow()}
     */
    public Action actionSetIntakeReverse() {
        return new InstantAction(intake::runBackwardsSlow); // slow reverse to eject partial artifacts
    }

    /**
     * Returns an instantaneous {@link Action} that sets the intake to slow
     * passive speed (keeps the intake alive without aggressive pulling).
     *
     * @return an {@link InstantAction} that calls {@code intake.runSlow()}
     */
    public Action actionSetIntakePassive() {
        return new InstantAction(intake::runSlow); // passive slow intake
    }

    // -----------------------------------------------------------------------
    // Timed intake trajectory
    // -----------------------------------------------------------------------

    /**
     * Returns an {@link Action} that drives the robot to the intake field zone,
     * runs the intake during the approach, and advances the indexer at
     * pre-calibrated time offsets to accept up to three artifacts.
     *
     * <p>The indexer advancement is time-based (not sensor-based); adjust
     * {@link #ball1TimeDisp} and {@link #ball2TimeDisp} to match the actual
     * spacing between field artifacts.
     *
     * @param startActionPose the robot pose at which to begin this action
     * @param startIntakePose the pose where the intake port aligns with the first artifact
     * @param endPose         the pose to return to after intake (typically shooting position)
     * @param drive           the Road Runner drive used to build the trajectory
     * @param maxVel          maximum translational velocity (in/s) for the return segment
     * @return the built trajectory {@link Action}
     */
    public Action actionIntakeThree(Pose2d startActionPose, Pose2d startIntakePose, Pose2d endPose, MecanumDrive drive, double maxVel) {
        // Limit return-segment velocity to maxVel so the robot doesn't overshoot.
        TranslationalVelConstraint velConstraint = new TranslationalVelConstraint(maxVel);

        return drive.actionBuilder(startActionPose)
                .strafeToSplineHeading(startIntakePose.position, startIntakePose.heading) // approach intake zone
                .afterTime(0, intake::run)                                                // start intake immediately
                .afterTime(ball1TimeDisp, () -> indexer.moveTo(indexer.getState().next())) // advance to slot 1 after first artifact time
                .afterTime(ball2TimeDisp, () -> indexer.moveTo(indexer.getState().next())) // advance to slot 2 after second artifact time
                .afterTime(timeToIntake, intake::runSlow)                                  // slow down intake at end of dwell time
                .strafeToLinearHeading(endPose.position, endPose.heading, velConstraint)   // return to shooting position
                .build();
    }

    // -----------------------------------------------------------------------
    // Auto initialization
    // -----------------------------------------------------------------------

    /**
     * Returns an {@link Action} that initializes the robot for autonomous:
     * clears all indexer slot colours, enables intake auto-advance at the
     * specified starting slot, lowers the actuator, and starts passive intake.
     *
     * <p>Note: this is a temporary action for testing; in a real auto run the
     * initialization is performed as part of {@link #actionQuickOuttake()}.
     *
     * @param indexerInitializedSlot the indexer slot to start intake at (ensures
     *                               consistent starting position regardless of where
     *                               the drum was left)
     * @return a {@link ParallelAction} that resets the indexer, lowers the actuator,
     *         and starts slow intake simultaneously
     */
    public Action initializeAuto(Indexer.IndexerState indexerInitializedSlot) { // only temporary for testing, this is done in actionQuickOuttake
        return new ParallelAction(
            new SequentialAction(
                new InstantAction(() -> indexer.initializeColors(Indexer.ArtifactColor.EMPTY)), // mark all slots empty
                new InstantAction(() -> indexer.setIntaking(true, indexerInitializedSlot))      // start intake at specified slot
            ),
            new InstantAction(actuator::down),   // lower the actuator to intake position
            new InstantAction(intake::runSlow)   // begin slow passive intake
        );
    }

    // -----------------------------------------------------------------------
    // Feedback-based intake
    // -----------------------------------------------------------------------

    /**
     * Returns an {@link Action} that drives to the intake zone and indexes
     * artifacts using sensor feedback rather than fixed time offsets.
     *
     * <p>The robot drives the trajectory (approach → return) while a parallel
     * action monitors the indexer sensor.  Each time the sensor detects an
     * artifact aligned with the intake port (rising edge), the indexer advances
     * to the next slot after a cooldown period.  The parallel action ends when
     * three artifacts have been acquired or the drive trajectory completes.
     *
     * <p>Compared to {@link #actionIntakeThree}, this is more robust to
     * varying artifact positions because advancement is triggered by detection
     * rather than elapsed time.
     *
     * @param startActionPose starting pose for the trajectory
     * @param startIntakePose pose where the robot begins actively intaking
     * @param endPose         return pose after intake
     * @param drive           Road Runner drive for trajectory execution
     * @param maxVel          maximum return velocity (in/s)
     * @return a {@link SequentialAction}: start intake → parallel(drive, feedback-index) → slow intake
     */
    //feedback based version of actionIntakeT
    public Action actionIntakeThreeFeedback(
            Pose2d startActionPose,
            Pose2d startIntakePose,
            Pose2d endPose,
            MecanumDrive drive,
            double maxVel
    ) {
        // Limit return-segment velocity so the robot doesn't overshoot the shoot position.
        VelConstraint velConstraint = new TranslationalVelConstraint(maxVel);

        // Build the drive trajectory: approach intake zone, then return to end pose.
        Action driveAction = drive.actionBuilder(startActionPose)
                .strafeToSplineHeading(startIntakePose.position, startIntakePose.heading) // approach
                .strafeToLinearHeading(endPose.position, endPose.heading, velConstraint)  // return
                .build();

        // Parallel action that manages intake speed and indexer advancement based on sensor.
        Action manageIntakeAndIndexing = new Action() {
            private boolean lastAlignedNonEmpty = false; // sensor state from the previous iteration
            private int acquired = 0;                    // number of artifacts acquired so far (max 3)
            private double timeoutDuration = 5.0;        // (unused) fallback timeout in seconds
            private final ElapsedTime acquireCooldown = new ElapsedTime(); // cooldown between advancements

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                // Abort if the OpMode has been stopped (e.g., end of auto period).
                if (!opMode.opModeIsActive() || opMode.isStopRequested()) {
                    intake.runSlow(); // leave intake in slow mode to avoid sharp stop
                    return false;     // signal that this action is done
                }

                // Tick the indexer so its internal PD control and sensor reads stay current.
                indexer.update();

                // Check if an artifact is currently aligned with the intake port AND present.
                boolean alignedNonEmpty = indexer.artifactPresentAndAligned();

                // Modulate intake speed: slow down once an artifact is detected to prevent
                // the next one from jamming while the drum rotates.
                if (alignedNonEmpty) intake.runSlow(); // slow: artifact present, wait for indexer to advance
                else                 intake.run();     // full speed: no artifact yet, keep pulling

                // Rising-edge detection: only count when the sensor just became true.
                // Also enforce the cooldown so a single artifact can't be counted twice.
                if (!lastAlignedNonEmpty
                        && alignedNonEmpty
                        && acquireCooldown.milliseconds() > cooldownFeedbackIntake) {

                    acquired++;              // increment the acquired count
                    acquireCooldown.reset(); // restart the cooldown timer

                    if (acquired < 3) {
                        // Advance to the next slot to make room for the next artifact.
                        indexer.moveTo(indexer.getState().next());
                    }
                }

                lastAlignedNonEmpty = alignedNonEmpty; // store for rising-edge detection next iter

                // End condition: all three slots are filled.
                if (acquired >= 3) {
                    intake.runSlow(); // transition to slow intake to avoid jamming the third ball
                    return false;     // signal that intake is complete
                }

                // Write debug data to the telemetry packet for the FTC Dashboard.
                p.put("acquired", acquired);
                p.put("alignedNonEmpty", alignedNonEmpty);
                p.put("indexerState", indexer.getState());

                return true; // still collecting; keep running this iteration
            }
        };

        // Wire everything together: start intake, then drive and index in parallel, then slow.
        return new SequentialAction(
                new InstantAction(intake::run),       // start intake before entering the parallel block

                new ParallelAction(
                        driveAction,              // drive the intake trajectory
                        manageIntakeAndIndexing   // concurrently manage intake speed & indexer
                ),

                new InstantAction(intake::runSlow)    // after both finish, settle into slow intake
        );
    }

    // -----------------------------------------------------------------------
    // Obelisk scan
    // -----------------------------------------------------------------------

    /**
     * Returns an {@link Action} that continuously calls the Limelight to scan
     * for the obelisk AprilTag and terminates as soon as a valid motif tag
     * (ID 21, 22, or 23) is detected.
     *
     * <p>The detected ID is stored in {@link #obeliskId} for later retrieval
     * via {@link #getObeliskId()} or via the {@link AprilTag#obeliskId} static
     * field.
     *
     * <p><b>Note:</b> Running this as an {@link InstantAction} and wrapping with
     * a while loop is not ideal (the action blocks the Road Runner scheduler on
     * the same thread); this is a known issue noted in the source.
     *
     * @return an {@link Action} that loops until a valid obelisk tag is found
     */
    // bad to do instant action and while loop
    public Action actionScanObelisk() {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime(); // (unused) reserved for timeout

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Scan the limelight for the obelisk tag on pipeline 2.
                aprilTag.scanObeliskTag();
                obeliskId = aprilTag.getObeliskId(); // cache locally for retrieval

                // Stop scanning once a valid motif tag is found.
                if (obeliskId == 21 || obeliskId == 22 || obeliskId == 23)
                    return false; // valid tag found: action complete

                return true; // still searching: run again next iteration
            }
        };
    }

    // -----------------------------------------------------------------------
    // Periodic background action
    // -----------------------------------------------------------------------

    /**
     * Returns a continuous background {@link Action} that runs every loop
     * iteration to keep subsystems updated.
     *
     * <p>This action is designed to be raced or run in parallel with trajectory
     * actions so that the indexer, outtake RPM controller, and pose estimator
     * remain current even while Road Runner is executing a path.
     *
     * <p>The action terminates when the OpMode is stopped.
     *
     * @return a looping {@link Action} that ticks pose estimation, outtake, and indexer
     */
    public Action actionPeriodic() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                // Stop if the OpMode ends so this action doesn't outlive the OpMode.
                if (!opMode.opModeIsActive() || opMode.isStopRequested()) {
                    return false; // terminate the periodic action
                }

                // Tick all subsystems that need per-loop updates.
                drive.updatePoseEstimate(); // integrate odometry/IMU into the pose estimate
                outtake.periodic();         // update flywheel RPM measurement and controller
                indexer.update();           // run indexer PD control and color sensing

                telemetry.addData("obelisk id: ", obeliskId); // show last obelisk ID on dashboard
                telemetry.update();

                // If continuous heading lock is enabled, scan the goal tag and compute correction.
                if (continuousAprilTagLock) {
                    aprilTag.scanGoalTag(); // update bearing from Limelight
                    double bearing = aprilTag.getBearing();

                    double turn = 0;
                    if (!Double.isNaN(bearing)) {
                        // Compute turn correction but (currently) not applied to trajectory.
                        turn = aprilAimer.calculateTurnPowerFromBearing(bearing);
                    }
                }

                return true; // still active; run again next iteration
            }
        };
    }

    // -----------------------------------------------------------------------
    // Park action
    // -----------------------------------------------------------------------

    /**
     * Returns an {@link Action} representing the end-of-auto parking sequence.
     * Currently a placeholder (1-second sleep); future implementation will
     * raise vertical slides or drive to the parking zone.
     *
     * @return a parking {@link Action} (placeholder)
     */
    public Action actionPark() {
    // vert slides
        return new ParallelAction(
                new SleepAction(1) // placeholder: 1-second wait for park position
        );
    }

    // -----------------------------------------------------------------------
    // Accessors
    // -----------------------------------------------------------------------

    /**
     * Returns the most recently detected obelisk AprilTag ID.
     * Valid motif IDs are 21, 22, and 23.  Returns 0 if no tag has been detected.
     *
     * @return the obelisk tag ID, or 0 if not yet detected
     */
    public int getObeliskId() {
        return obeliskId; // most recently scanned obelisk AprilTag ID
    }

    // -----------------------------------------------------------------------
    // Private helpers
    // -----------------------------------------------------------------------

    /**
     * Checks whether the indexer slot at {@code stateIndex} and the next slot
     * (modulo 3) match the first two colours of {@code desired}.
     *
     * <p>Used by {@link #rotateToMotifColorBeforeOuttake} to find the starting
     * rotation that aligns with the desired firing order.
     *
     * @param stateIndex zero-based indexer slot index to check
     * @param desired    the desired shooting order (must have at least 2 elements)
     * @return {@code true} if slot[stateIndex] == desired[0] AND slot[(stateIndex+1)%3] == desired[1]
     */
    private boolean matchesOrder(int stateIndex, Indexer.ArtifactColor[] desired) {
        // Check the first slot against desired[0] and the next slot (wrapping) against desired[1].
        return indexer.getColorAt(Indexer.IndexerState.values()[stateIndex % 3]) == desired[0]
                && indexer.getColorAt(Indexer.IndexerState.values()[(stateIndex + 1) % 3]) == desired[1];
    }


    // HELPERS

    /**
     * Applies per-row colour assignments to the indexer, rotated so that the
     * first intaken artifact lands in {@code startingSlot}.
     *
     * <p>Row-to-colour mapping (based on field artifact placement):
     * <ul>
     *   <li>Row 1 → Purple, Purple, Green</li>
     *   <li>Row 2 → Purple, Green, Purple</li>
     *   <li>Row 0 or 3 → Green, Purple, Purple</li>
     * </ul>
     *
     * <p>The rotation ensures that if the first artifact is intaken into slot
     * {@code startingSlot}, the subsequent slots are assigned colours in the
     * order they will be collected.
     *
     * @param row          field row number (0–3); determines the colour pattern
     * @param startingSlot the indexer slot index where the first intaken artifact
     *                     will land (basically {@code (lastMovedSlot + 1) % 3} in intake mode)
     */
    // Sets the indexer's color configuration based on a given row,
    // rotated so that the first intaken ball is placed in startingSlot
    private void applyCurrentColorsFromRow(int row, int startingSlot /* basically (the last moved to slot + 1) % 3 [in intaking mode]*/) {
        Indexer.ArtifactColor[] intakeOrder; // colour order for this row (index 0 = first intaken)

        // Map the row number to the known artifact color sequence for that row.
        switch (row) {
            case 1: // Row 1 artifact pattern: Purple, Purple, Green
                intakeOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.GREEN};
                break;

            case 2: // Row 2 artifact pattern: Purple, Green, Purple
                intakeOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE};
                break;

            case 0:
            case 3: // Row 0 or 3 artifact pattern: Green, Purple, Purple
                intakeOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.PURPLE};
                break;

            default:
                return; // unknown row: do not change the indexer colour state
        }

        // Rotate the colour array so the first intaken artifact lands in startingSlot.
        // rotated[startingSlot + i] = intakeOrder[i]  (modulo 3)
        Indexer.ArtifactColor[] rotated = new Indexer.ArtifactColor[3];
        for (int i = 0; i < 3; i++) {
            rotated[(startingSlot + i) % 3] = intakeOrder[i]; // place each colour in its rotated slot
        }

        // Apply the rotated colour assignments to the indexer's internal colour table.
        indexer.initializeColors(rotated[0], rotated[1], rotated[2]);
    }

    /**
     * Converts an obelisk AprilTag ID to the corresponding desired artifact
     * shooting order (which colour should be fired first, second, and third).
     *
     * <p>Tag-to-order mapping:
     * <ul>
     *   <li>21 → Green, Purple, Purple</li>
     *   <li>22 → Purple, Green, Purple</li>
     *   <li>23 → Purple, Purple, Green</li>
     * </ul>
     *
     * @param id the obelisk AprilTag ID (should be 21, 22, or 23)
     * @return an array of three {@link Indexer.ArtifactColor} values representing
     *         the desired firing order; empty array if the ID is unrecognised
     */
    private Indexer.ArtifactColor[] getDesiredShootOrder(int id) {
        // After the tag ID cases, you want to change the physical rows into shooting that motif
        Indexer.ArtifactColor[] desiredShootOrder;
        switch (id) {
            case 21: // Tag 21 → G -> P -> P - will shoot out in this order
                desiredShootOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.PURPLE};
                break;

            case 22: // Tag 22 → P -> G -> P
                desiredShootOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE};
                break;

            case 23: // Tag 23 → P -> P -> G
                desiredShootOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.GREEN};
                break;

            default:
                desiredShootOrder = new Indexer.ArtifactColor[] {}; // unknown ID: empty order
                break;
        }

        return desiredShootOrder; // return the color order to fire in
    }
}
