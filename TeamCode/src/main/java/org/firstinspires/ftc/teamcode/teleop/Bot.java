package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

/**
 * Bot is the primary teleop controller for the robot.
 *
 * <p>It extends {@link BotPeriodics} (which owns the hardware and runs all
 * per-loop subsystem updates) and adds a finite-state machine (FSM) for
 * managing the five phases of a teleop match:
 *
 * <ol>
 *   <li><b>MotifSelection</b> – drivers choose the desired scoring motif
 *       (color order) before entering the Intake phase.</li>
 *   <li><b>Intake</b> – robot collects three artifacts via the intake/indexer
 *       while the driver pre-spins the shooter when ready.</li>
 *   <li><b>QuickOuttake</b> – full-drum "quick dump" mode: shooter is spun up
 *       and the entire drum is emptied at once in a timed blast.</li>
 *   <li><b>SortOuttake</b> – sorted single-ball mode: the driver selects the
 *       colour of the next artifact to fire and the indexer rotates to the
 *       correct slot before each shot.</li>
 *   <li><b>Endgame</b> – reserved for hanging or end-game actions.</li>
 * </ol>
 *
 * <p>Annotated with {@link Config} so tuning constants are visible in Dashboard.
 */
@Config
public class Bot extends BotPeriodics {

    // -----------------------------------------------------------------------
    // Haptics / lights state
    // -----------------------------------------------------------------------

    /** Latched to {@code true} once the "drum full" rumble has been sent to
     *  prevent repeated rumbles every loop tick after the drum fills. */
    private boolean rumbledAlready = false;

    // -----------------------------------------------------------------------
    // Finite-state machine
    // -----------------------------------------------------------------------

    /**
     * The five high-level states of the teleop FSM.
     */
    public enum FSM {
        /** Pre-match: drivers select which scoring motif (color pattern) to use. */
        MotifSelection,
        /** Collecting artifacts via the intake/indexer assembly. */
        Intake,
        /** Full-drum timed dump: all three artifacts fired in one blast. */
        QuickOuttake,
        /** Sorted single-ball outtake: fire one artifact at a time by color. */
        SortOuttake,
        /** End-game phase (climbing, parking, or special actions). */
        Endgame
    }

    /** Current FSM state.  Drives the {@code switch} in {@link #teleopTick()}. */
    public FSM state;

    // -----------------------------------------------------------------------
    // Tuning constants (Dashboard-editable)
    // -----------------------------------------------------------------------

    /** Duration (seconds) of the full-power "non-indexed dump" spin in
     *  {@link #actionNonIndexedDump()}.  Long enough to clear all three slots
     *  but short enough not to waste time after the drum is empty. */
    public static double NON_INDEX_SPIN_TIME = 3; // seconds

    /** Raw power applied to the indexer drum during the non-indexed dump blast.
     *  Low enough to feed balls smoothly without jamming. */
    public static double FULL_BLAST_POWER = 0.25;

    /** Scale factor applied to the base shooter RPM during the quick-spin outtake.
     *  Values < 1.0 reduce RPM slightly to trade accuracy for faster spin-up. */
    public static double QUICKSPIN_OUTTAKE_RPM_SCALE = 0.93; // proportion of full target RPM

    // -----------------------------------------------------------------------
    // Motif definitions
    // -----------------------------------------------------------------------

    /** The motif (desired color order of artifacts) currently selected by the driver.
     *  Determines which slot the indexer presents first during outtake. */
    public Indexer.ArtifactColor[] motif;

    /** Purple–Purple–Green motif: two purple balls then one green. */
    private Indexer.ArtifactColor[] PPG = new Indexer.ArtifactColor[]{
        Indexer.ArtifactColor.PURPLE,
        Indexer.ArtifactColor.PURPLE,
        Indexer.ArtifactColor.GREEN
    };

    /** Purple–Green–Purple motif: purple, green, purple interleaved. */
    private Indexer.ArtifactColor[] PGP = new Indexer.ArtifactColor[]{
        Indexer.ArtifactColor.PURPLE,
        Indexer.ArtifactColor.GREEN,
        Indexer.ArtifactColor.PURPLE
    };

    /** Green–Purple–Purple motif: green first, then two purple. */
    private Indexer.ArtifactColor[] GPP = new Indexer.ArtifactColor[]{
        Indexer.ArtifactColor.GREEN,
        Indexer.ArtifactColor.PURPLE,
        Indexer.ArtifactColor.PURPLE
    };

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Constructs a Bot, passing all hardware references to the parent class
     * and initialising the FSM in the {@link FSM#MotifSelection} state.
     *
     * @param hardwareMap   the robot's hardware map
     * @param tele          FTC telemetry
     * @param mecanumDrive  pre-built Road Runner drive
     * @param gamepad1      Driver 1 raw gamepad
     * @param gamepad2      Driver 2 (operator) raw gamepad
     * @param twoMovement   {@code true} to allow gamepad 2 to also control driving
     */
    public Bot(HardwareMap hardwareMap, Telemetry tele, MecanumDrive mecanumDrive, Gamepad gamepad1, Gamepad gamepad2, boolean twoMovement) {
        super(hardwareMap, tele, mecanumDrive, gamepad1, gamepad2, twoMovement);
        state = FSM.MotifSelection; // always start in motif-selection so drivers confirm the motif
    }

    // -----------------------------------------------------------------------
    // OpMode lifecycle
    // -----------------------------------------------------------------------

    /**
     * Called once during the {@code init()} phase, before the start button is
     * pressed.  Resets the indexer color table, enables auto intake advancement,
     * and stops the shooter.
     */
    public void teleopInit() {
        indexer.initializeColors(Indexer.ArtifactColor.EMPTY); // mark every slot as empty at start
        indexer.setIntaking(true);   // enable the indexer's auto-advance-on-intake logic
        state = FSM.MotifSelection;  // reset FSM to motif-selection in case of re-init
        outtake.stop();              // ensure shooter is off before the match begins
    }

    /**
     * Called once immediately after the start button is pressed.
     * Lowers the actuator to the "down / intake" position and moves the
     * indexer to its zero (home) slot so intake can begin immediately.
     */
    public void teleopStart(){
        actuator.down();                               // bring the actuator to the loading position
        indexer.moveTo(Indexer.IndexerState.zero);     // rotate drum so slot 0 faces the intake port
    }

    // -----------------------------------------------------------------------
    // Main loop tick
    // -----------------------------------------------------------------------

    /**
     * Called every loop iteration from the OpMode's while loop.
     * Runs all periodic subsystem updates first, then delegates to the
     * appropriate FSM state handler.
     */
    public void teleopTick()
    {
        handlePeriodics(); // run all subsystem periodic updates (inherited from BotPeriodics)

        // Route to the correct state handler based on the current FSM state.
        switch (state) {
            case MotifSelection:
                // Wait for the driver to press a face button choosing the motif.
                if (g2.wasJustPressed(GamepadKeys.Button.X)){
                    motif = PPG;                      // select PPG motif
                    indexer.prepareQuickspin(motif);  // rotate drum to the first motif slot
                    state = FSM.Intake;               // transition to intake phase
                }
                if (g2.wasJustPressed(GamepadKeys.Button.Y)){
                    motif = PGP;                      // select PGP motif
                    indexer.prepareQuickspin(motif);
                    state = FSM.Intake;
                }
                if (g2.wasJustPressed(GamepadKeys.Button.B)){
                    motif = GPP;                      // select GPP motif
                    indexer.prepareQuickspin(motif);
                    state = FSM.Intake;
                }
                break;
            case Intake:
                handleIntakeState();      // handle intake controls and state transitions
                break;
            case QuickOuttake:
                handleQuickOuttakeState(); // handle quick-dump controls
                break;
            case SortOuttake:
                handleSortOuttakeState(); // handle sorted single-ball firing controls
                break;
            case Endgame:
                handleEndgameState();     // handle endgame controls
                break;
        }
    }

    // -----------------------------------------------------------------------
    // FSM state handlers
    // -----------------------------------------------------------------------

    /**
     * Handles driver inputs and logic while in the {@link FSM#Intake} state.
     *
     * <p>Key bindings (gamepad 2):
     * <ul>
     *   <li>Left trigger – run intake at full speed</li>
     *   <li>Right bumper (hold) – pre-spin shooter, transition to QuickOuttake</li>
     *   <li>A – jump directly to QuickOuttake state</li>
     *   <li>B – jump to SortOuttake state (disable intake auto-advance)</li>
     *   <li>Y – jump to Endgame state</li>
     *   <li>DPAD_RIGHT – manually advance the indexer to the next slot</li>
     *   <li>DPAD_UP – re-prepare the quick-spin order for the current motif</li>
     *   <li>X – manually set all slots to PPG colours (debug override)</li>
     * </ul>
     *
     * <p>Also handles the "drum full" rumble notification to both gamepads.
     */
    // MAINLINE HANDLERS
    private void handleIntakeState() {
        double leftTrigger = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        // Right bumper held: begin spinning up the shooter and transition to quick outtake.
        if (!actionHost.isRunning()) {
            if (g2.gamepad.right_bumper) {
                rangeRequested = true;       // start requesting range / turn-correction from aimer
                state = FSM.QuickOuttake;    // switch FSM to quick-outtake mode
                applyPreSpinRPM();           // command shooter to pre-spin target RPM
            } else {
                outtake.stop();              // no bumper held: ensure shooter is off
                rangeRequested = false;      // stop requesting range updates
            }
        }

        // Left trigger: manually run the intake in addition to any continuous intake.
        if (leftTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE) intake.run();
        else intake.stop();

        // DPAD_RIGHT: manually step the indexer to the next slot.
        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            indexer.moveTo(indexer.getState().next());

        // X: debug override – force all three slots to the PPG colour assignment.
        if(g2.wasJustPressed(GamepadKeys.Button.X))
            indexer.initializeColors(
                Indexer.ArtifactColor.PURPLE,
                Indexer.ArtifactColor.PURPLE,
                Indexer.ArtifactColor.GREEN
            );

        // A: jump to QuickOuttake without pre-spinning.
        if (g2.wasJustPressed(GamepadKeys.Button.A))
            state = FSM.QuickOuttake;

        // B: jump to SortOuttake (disable intake auto-advance so drum doesn't rotate during aiming).
        if (g2.wasJustPressed(GamepadKeys.Button.B)){
            state = FSM.SortOuttake;
            indexer.setIntaking(false);       // stop indexer auto-advance
            indexer.moveTo(indexer.getState()); // hold current slot
        }

        // DPAD_UP: re-prepare the quick-spin ordering for the current motif.
        if(g2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            indexer.prepareQuickspin(new Indexer.ArtifactColor[]{
                Indexer.ArtifactColor.GREEN,
                Indexer.ArtifactColor.PURPLE,
                Indexer.ArtifactColor.PURPLE
            });

        // Y: jump to endgame mode.
        if (g2.wasJustPressed(GamepadKeys.Button.Y)) state = FSM.Endgame;

        // Second DPAD_UP check prepares using the driver-selected motif.
        if(g2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            indexer.prepareQuickspin(motif);

        // Drum-full notification: rumble both gamepads once when the drum fills.
        if(indexer.isFull() && !rumbledAlready
                && !g1.gamepad.isRumbling() && !g2.gamepad.isRumbling()){
            g1.gamepad.rumbleBlips(TeleopConstants.Gamepad.FULL_WARNING_RUMBLES); // alert driver 1
            g2.gamepad.rumbleBlips(TeleopConstants.Gamepad.FULL_WARNING_RUMBLES); // alert driver 2
            rumbledAlready = true; // latch so we only rumble once per fill cycle
        }
    }

    /**
     * Overrides the parent's alliance selection handler to add gamepad LED
     * color feedback when an alliance is selected.
     *
     * <p>Key bindings (gamepad 1):
     * <ul>
     *   <li>{@code BACK} – select blue alliance; set gamepad LED to blue</li>
     *   <li>{@code START} – select red alliance; set gamepad LED to red</li>
     * </ul>
     */
    protected void handleAllianceSelection() {
        if (g1.wasJustPressed(GamepadKeys.Button.BACK)) {
            aprilTag.setPipeline(0);  // pipeline 0 = blue alliance goal tag
            aimer.setBlueTarget();
            // Set gamepad 1 LED to blue for the configured duration.
            g1.gamepad.setLedColor(0, 0, 1, TeleopConstants.Gamepad.GAMEPAD_LIGHT_COLOR_DURATION);
            colorGoalSelected = "Blue";
        }
        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            aprilTag.setPipeline(1);  // pipeline 1 = red alliance goal tag
            aimer.setRedTarget();
            // Set gamepad 1 LED to red for the configured duration.
            g1.gamepad.setLedColor(1, 0, 0, TeleopConstants.Gamepad.GAMEPAD_LIGHT_COLOR_DURATION);
            colorGoalSelected = "Red";
        }
    }

    /**
     * Handles driver inputs while in the {@link FSM#QuickOuttake} state.
     *
     * <p>Key bindings (gamepad 2):
     * <ul>
     *   <li>Right bumper (hold) – continue pre-spinning the shooter</li>
     *   <li>X – execute the full non-indexed dump action; then return to Intake</li>
     *   <li>DPAD_UP – re-prepare the quick-spin order for the current motif</li>
     *   <li>BACK – abort any running action</li>
     *   <li>A – cancel quick outtake; return to Intake</li>
     * </ul>
     */
    private void handleQuickOuttakeState() {
        // Allow press-and-hold pre-spin while waiting to confirm the dump.
        if (!actionHost.isRunning()) {
            if (g2.gamepad.right_bumper) {
                applyPreSpinRPM();   // keep shooter at pre-spin RPM
                rangeRequested = true; // keep requesting range/turn correction
            } else {
                rangeRequested = false;
                outtake.stop();      // bumper released: stop shooter
            }
        }

        // X: fire the non-indexed dump action; reset rumble latch and return to Intake after.
        if (!actionHost.isRunning() && g2.wasJustPressed(GamepadKeys.Button.X)) {
            actionHost.start(actionNonIndexedDump()); // start the multi-step dump sequence
            rumbledAlready = false;   // allow re-triggering the rumble for the next fill cycle
            state = FSM.Intake;       // FSM transitions back to Intake when the action finishes
        }

        // DPAD_UP: re-prepare the quick-spin ordering for the current motif.
        if(g2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            indexer.prepareQuickspin(motif);

        // BACK: abort whatever action is currently running.
        if (g2.wasJustPressed(GamepadKeys.Button.BACK)) {
            actionHost.abort(); // cancel the in-progress action immediately
        }

        // A: cancel quick outtake; re-enable intake auto-advance and return to Intake.
        if (g2.wasJustPressed(GamepadKeys.Button.A)) {
            state = FSM.Intake;
            indexer.setIntaking(true); // re-enable auto-advance so new artifacts can be indexed
            rumbledAlready = false;    // reset rumble latch
        }
    }

    /**
     * Handles driver inputs while in the {@link FSM#SortOuttake} state.
     *
     * <p>Key bindings (gamepad 2):
     * <ul>
     *   <li>X – fire the next GREEN artifact</li>
     *   <li>Y – fire the next PURPLE artifact</li>
     *   <li>BACK – abort any running action</li>
     *   <li>A – return to Intake; re-enable intake auto-advance</li>
     * </ul>
     */
    private void handleSortOuttakeState() {
        if (!actionHost.isRunning()) {
            // X: fire the next available green artifact.
            if (g2.wasJustPressed(GamepadKeys.Button.X)) {
                actionHost.start(actionFireGreen()); // start green-shot sequence
            }
            // Y: fire the next available purple artifact.
            if (g2.wasJustPressed(GamepadKeys.Button.Y)) {
                actionHost.start(actionFirePurple()); // start purple-shot sequence
            }
        }
        // BACK: abort any currently running shot sequence.
        if (g2.wasJustPressed(GamepadKeys.Button.BACK)) {
            actionHost.abort(); // cancel the in-progress shot action
        }
        // A: exit sort-outtake; re-enable intake and return to collecting.
        if (g2.wasJustPressed(GamepadKeys.Button.A)) {
            indexer.setIntaking(true); // re-enable auto-advance on the indexer
            state = FSM.Intake;
        }
    }

    /**
     * Handles driver inputs while in the {@link FSM#Endgame} state.
     *
     * <p>Key bindings (gamepad 2):
     * <ul>
     *   <li>A – return to Intake and re-enable intake auto-advance</li>
     * </ul>
     */
    private void handleEndgameState() {
        // A: leave endgame and resume normal intake.
        if (g2.wasJustPressed(GamepadKeys.Button.A)) {
            state = FSM.Intake;
            indexer.setIntaking(true); // re-enable auto-advance so the drum can fill again
        }
    }

    // -----------------------------------------------------------------------
    // RPM helper
    // -----------------------------------------------------------------------

    /**
     * Commands the shooter to spin up to the "quick-spin" RPM (base target RPM
     * scaled by {@link #QUICKSPIN_OUTTAKE_RPM_SCALE}).
     * Called while the driver holds the right bumper to pre-heat the flywheel.
     */
    private void applyPreSpinRPM() {
        outtake.set(getTargetRPM() * QUICKSPIN_OUTTAKE_RPM_SCALE); // RPM mode: set shooter target RPM
    }

    // -----------------------------------------------------------------------
    // Action builders
    // -----------------------------------------------------------------------

    /**
     * Builds the "non-indexed dump" Road Runner {@link Action} sequence.
     *
     * <p>This is the quick-outtake path: all three artifacts are fired at once
     * without sorting by colour.  The sequence:
     * <ol>
     *   <li>Raise actuator to the "quick" (lower) up position.</li>
     *   <li>Set shooter to quick-spin RPM.</li>
     *   <li>Wait until the shooter is within 50 RPM of target.</li>
     *   <li>Run the indexer drum at {@link #FULL_BLAST_POWER} for
     *       {@link #NON_INDEX_SPIN_TIME} seconds to dump all artifacts.</li>
     *   <li>Stop the indexer and shooter.</li>
     *   <li>Lower the actuator.</li>
     *   <li>Re-enable intake auto-advance.</li>
     *   <li>Reset all slot colours to EMPTY.</li>
     *   <li>Return the indexer to slot zero ready for the next intake cycle.</li>
     * </ol>
     *
     * @return the fully-constructed dump {@link Action}
     */
    private Action actionNonIndexedDump() {
        final double rpm = getTargetRPM() * QUICKSPIN_OUTTAKE_RPM_SCALE; // pre-compute RPM to use
        return new SequentialAction(
                new InstantAction(actuator::upQuick),                         // 1. raise actuator quickly
                new InstantAction(() -> outtake.set(rpm)),                    // 2. start shooter spin-up
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        return !outtake.inRange(50); // 3. block until RPM is within 50 of target
                    }
                },
                new InstantAction(() -> indexer.setIndexerPower(FULL_BLAST_POWER)), // 4. blast drum
                new SleepAction(NON_INDEX_SPIN_TIME),                              // 5. timed blast duration
                new InstantAction(indexer::stopIndexerPower),                      // 6. stop drum
                new InstantAction(outtake::stop),                                  // 7. stop shooter
                new InstantAction(actuator::down),                                 // 8. lower actuator
                new InstantAction(() -> indexer.setIntaking(true)),                // 9. re-enable intake
                new InstantAction(indexer::initializeColors),                      // 10. mark all slots EMPTY
                new InstantAction(() -> indexer.moveTo(Indexer.IndexerState.zero)) // 11. home indexer
        );
    }

    /**
     * Builds the "fire green artifact" Road Runner {@link Action} sequence.
     *
     * <p>Finds the best slot containing a GREEN artifact, rotates the indexer
     * to that slot, spins up the shooter, and fires when in range.
     * If no green artifact is loaded, returns a no-op action.
     *
     * <p>Sequence:
     * <ol>
     *   <li>Disable intake auto-advance.</li>
     *   <li>Lower the actuator.</li>
     *   <li>Rotate indexer to the green slot.</li>
     *   <li>Spin up shooter to target RPM.</li>
     *   <li>Wait until within 100 RPM of target.</li>
     *   <li>Raise actuator to indexed position.</li>
     *   <li>Wait 1 second for the artifact to exit.</li>
     *   <li>Mark that slot as EMPTY.</li>
     *   <li>Stop shooter and lower actuator.</li>
     * </ol>
     *
     * @return the green-shot {@link Action}, or a no-op if no green slot exists
     */
    private Action actionFireGreen() {
        // Find the indexer slot that best matches a GREEN artifact.
        final Indexer.IndexerState slot =
                indexer.findBestSlotForColor(Indexer.ArtifactColor.GREEN);

        if (slot == null) {
            // No green artifact loaded: return an empty no-op action.
            return new InstantAction(() -> {});
        }

        final double rpm = getTargetRPM(); // read target RPM now, not at action-run time

        return new SequentialAction(
                new InstantAction(() -> indexer.setIntaking(false)),         // 1. stop auto-advance
                new InstantAction(actuator::down),                           // 2. lower actuator (clear path)
                new InstantAction(() -> indexer.moveTo(slot, true)),         // 3. rotate to green slot
                new InstantAction(() -> outtake.set(rpm)),                   // 4. spin up shooter
                // 5. wait for shooter to reach target RPM within 100 RPM
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket p) {
                        return !outtake.inRange(100.0); // returns true while NOT in range (keep waiting)
                    }
                },
                new InstantAction(actuator::upIndexed),                      // 6. raise actuator to fire position
                new SleepAction(1),                                          // 7. allow 1 s for artifact to exit

                // 8. mark the fired slot as empty so the indexer knows it's clear
                new InstantAction(() ->
                        indexer.assignSlotColor(slot, Indexer.ArtifactColor.EMPTY)
                ),

                new InstantAction(outtake::stop),                            // 9. stop shooter
                new InstantAction(actuator::down)                            // 10. lower actuator back to intake pos
        );
    }

    /**
     * Builds the "fire purple artifact" Road Runner {@link Action} sequence.
     *
     * <p>Finds the best slot containing a PURPLE artifact, rotates the indexer
     * to that slot, spins up the shooter, and fires when in range.
     * If no purple artifact is loaded, returns a no-op action.
     *
     * <p>Sequence is the same as {@link #actionFireGreen()} but targets PURPLE.
     *
     * @return the purple-shot {@link Action}, or a no-op if no purple slot exists
     */
    private Action actionFirePurple() {
        // Find the indexer slot that best matches a PURPLE artifact.
        final Indexer.IndexerState slot =
                indexer.findBestSlotForColor(Indexer.ArtifactColor.PURPLE);

        if (slot == null) {
            // No purple artifact loaded: return an empty no-op action.
            return new InstantAction(() -> {
            });
        }

        final double rpm = getTargetRPM(); // capture current RPM target

        return new SequentialAction(
                new InstantAction(actuator::down),                           // 1. ensure actuator is lowered
                new InstantAction(() -> indexer.moveTo(slot, true)),         // 2. rotate to purple slot

                new InstantAction(() -> outtake.set(rpm)),                   // 3. spin up shooter
                // 4. wait for shooter RPM to be within 100 of target
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket p) {
                        return !outtake.inRange(100.0); // returns true while NOT in range (keep waiting)
                    }
                },
                new InstantAction(actuator::upIndexed),                      // 5. raise actuator to fire position
                new SleepAction(1),                                          // 6. allow 1 s for artifact to exit

                // 7. mark the fired slot as empty
                new InstantAction(() ->
                        indexer.assignSlotColor(slot, Indexer.ArtifactColor.EMPTY)
                ),

                new InstantAction(outtake::stop),                            // 8. stop shooter
                new InstantAction(actuator::down)                            // 9. lower actuator
        );
    }
}