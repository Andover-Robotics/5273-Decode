package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.indexerUtil.CRServoPositionControl;
import org.firstinspires.ftc.teamcode.subsystems.indexerUtil.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.subsystems.indexerUtil.SlotState;

/**
 * Indexer manages the three-slot rotating drum that holds and classifies game
 * artifacts (game elements) before they are fired by the shooter.
 *
 * <h2>Mechanical Overview</h2>
 * <p>The drum is a circular magazine with three equally-spaced slots (0, 1, 2)
 * separated by 120° of rotation.  A continuous-rotation servo
 * ({@link CRServoPositionControl}) drives the drum; an analog encoder tracks its
 * absolute angle.  A single colour sensor ({@link ColorSensorSystem}) is mounted
 * at a fixed position; as the drum rotates, each slot passes in front of the sensor.
 *
 * <h2>Colour Classification</h2>
 * <p>Each slot is assigned one of four colours:
 * {@link ArtifactColor#GREEN}, {@link ArtifactColor#PURPLE},
 * {@link ArtifactColor#EMPTY} (no artifact), or
 * {@link ArtifactColor#UNKNOWN} (present but hue is ambiguous).
 * Classification is probabilistic: multiple sensor reads are accumulated in a
 * {@link org.firstinspires.ftc.teamcode.subsystems.indexerUtil.SlotObservation}
 * per slot; the winner must exceed a configurable threshold.
 *
 * <h2>Modes</h2>
 * <ul>
 *   <li><b>Intaking mode</b> ({@link #setIntaking}): the drum is oriented so
 *       the intake opening aligns with the sensor; artifacts enter from the field.
 *       Auto-advance can be enabled to automatically step to the next empty slot
 *       after each artifact is collected.</li>
 *   <li><b>Outtake mode</b>: the drum rotates 186° to line up with the shooter
 *       barrel.  Each slot is fired sequentially by the {@code actionQuickOuttake}
 *       in BotActions.</li>
 *   <li><b>Auto-outtake mode</b>: a finer-grained offset used during autonomous
 *       sequences where the drum must be precisely aligned with the shooter at a
 *       different mechanical angle than teleop.</li>
 * </ul>
 *
 * <h2>Auto-Advancement Policies</h2>
 * <ul>
 *   <li>{@link AutoAdvancement#DISABLED} – no automatic drum movement; caller
 *       drives everything explicitly.</li>
 *   <li>{@link AutoAdvancement#SEEK_EMPTY} – every loop iteration that the drum
 *       is aligned with the current slot and an empty slot exists, the drum
 *       advances to the nearest empty slot.</li>
 *   <li>{@link AutoAdvancement#QUICK_SEEK} – like SEEK_EMPTY but only triggers
 *       when the colour sensor currently detects an artifact (faster to use during
 *       active intaking).</li>
 * </ul>
 *
 * <p>Annotated with {@link Config} so all dashboard-tunable fields are editable
 * live in the FTC Dashboard.
 */
@Config
public class Indexer {

    // -----------------------------------------------------------------------
    // Enums
    // -----------------------------------------------------------------------

    /**
     * Represents the colour (or absence) of an artifact stored in a slot.
     * Used both as the output of the colour classifier and as the stored
     * state in each {@link SlotState}.
     */
    public enum ArtifactColor {
        /** The artifact is green (one of two colours used in this game). */
        GREEN,
        /** The artifact is purple (the other game colour). */
        PURPLE,
        /** No artifact is present in this slot. */
        EMPTY,
        /** An artifact is detected but its colour cannot be reliably determined
         *  (sensor quality gate failed or hue is outside both defined ranges). */
        UNKNOWN
    }

    /**
     * Names for each of the three drum positions.
     * Each state holds its integer {@link #index} (0, 1, 2) and provides
     * {@link #next()} and {@link #last()} helpers for traversing the ring.
     */
    public enum IndexerState {
        /** Slot at index 0 (drum home position). */
        zero(0),
        /** Slot at index 1 (120° from home). */
        one(1),
        /** Slot at index 2 (240° from home). */
        two(2);

        /** The integer index of this slot (0, 1, or 2). */
        public final int index;

        /**
         * Constructs an IndexerState with the given index.
         * @param index the position index of this slot
         */
        IndexerState(int index) { this.index = index; }

        /**
         * Returns the next slot in the drum (wrapping from 2 → 0).
         * @return the IndexerState immediately clockwise of this one
         */
        public IndexerState next() { return values()[(index + 1) % values().length]; }

        /**
         * Returns the previous slot in the drum (wrapping from 0 → 2).
         * @return the IndexerState immediately counter-clockwise of this one
         */
        public IndexerState last() { return values()[(index + values().length - 1) % values().length]; }
    }

    // -----------------------------------------------------------------------
    // Dashboard overrides
    // -----------------------------------------------------------------------

    /** When {@code true} (rising edge), advances the drum by one slot.
     *  Set from FTC Dashboard for manual testing without a gamepad. */
    public static boolean dashAdvance = false;

    /** Target slot index commanded from the FTC Dashboard.
     *  -1 = disabled; 0/1/2 forces the drum to that specific slot.
     *  Used to manually position the drum during tuning. */
    public static int dashTargetSlot = -1; // -1 = disabled; 0/1/2 = slot

    // -----------------------------------------------------------------------
    // Auto-advancement policy enum
    // -----------------------------------------------------------------------

    /**
     * Selects how (or whether) the indexer automatically advances to the next
     * empty slot during intaking.
     */
    public enum AutoAdvancement
    {
        /** Never auto-advance; caller controls drum position entirely. */
        DISABLED,
        /** Advance to the next empty slot every loop iteration while the drum
         *  is aligned and an empty slot exists. */
        SEEK_EMPTY,
        /** Same as SEEK_EMPTY but only triggers when the sensor detects an
         *  artifact (avoids unnecessary drum movement when the intake is idle). */
        QUICK_SEEK
    }

    // -----------------------------------------------------------------------
    // Mode flags (configurable per use case)
    // -----------------------------------------------------------------------

    /** Current auto-advancement policy for this session.
     *  Should be set in the Hardware or OpMode constructor before use. */
    public AutoAdvancement AUTO_ADVANCEMENT_MODE = AutoAdvancement.QUICK_SEEK;

    /** When {@code true}, enables the full-drum "unknown scan" logic that
     *  rotates to unclassified slots to attempt a colour re-read when the drum
     *  is full but some slots remain UNKNOWN.  Disable in autonomous to
     *  save time when the slot colours are already pre-known. */
    public boolean ENABLE_FULL_UNKNOWN_SCAN = true;

    /** When {@code true}, the sensor classification pipeline runs every loop
     *  iteration to update slot colours.  Setting to {@code false} disables
     *  all colour sensing (useful in autonomous where colours are pre-assigned). */
    public boolean SCAN_COLORS = true;

    // -----------------------------------------------------------------------
    // Auto-advance sensitivity (HARD vs SOFT)
    // -----------------------------------------------------------------------

    /**
     * Selects whether auto-advancement requires many sensor confirmations (HARD)
     * or few (SOFT) before accepting that an artifact has entered the slot.
     * SOFT is faster but may false-trigger; HARD is slower but more robust.
     */
    public enum AutoDetectMode { HARD, SOFT }

    /** Currently active auto-detect mode (dashboard tunable). */
    public static AutoDetectMode AUTO_DETECT_MODE = AutoDetectMode.SOFT;

    /** Number of consecutive sensor non-empty reads required in HARD mode
     *  before an artifact is considered fully seated in the current slot. */
    public static int HARD_NONEMPTY_HITS_TO_ADVANCE = 8;

    /** Number of consecutive sensor non-empty reads required in SOFT mode
     *  before an artifact is considered fully seated in the current slot. */
    public static int SOFT_NONEMPTY_HITS_TO_ADVANCE = 3;

    // -----------------------------------------------------------------------
    // Drum angle offsets (degrees)
    // -----------------------------------------------------------------------

    /** Base angle offset added to every slot's computed center angle.
     *  Calibrate so that {@code slot 0} aligns with the colour sensor when
     *  the encoder reads 0°. */
    public static double offsetAngle = 92.0;

    /** Additional angle offset applied when the indexer is in autonomous-outtake
     *  mode ({@link #autoOuttaking} == true).  Accounts for the difference between
     *  the intake sensor position and the shooter feed position during auto. */
    public static double autoOuttakeOffsetAngle = 0.0;

    /** Additional angle offset applied when the indexer is in teleop-outtake
     *  mode ({@link #intaking} == false and {@link #autoOuttaking} == false).
     *  Rotates the drum ~186° so the slot that was facing the intake now faces
     *  the shooter barrel. */
    public static double outtakeOffsetAngle = 186.0;

    // -----------------------------------------------------------------------
    // Slot geometry constants
    // -----------------------------------------------------------------------

    /** Angular distance (degrees) between consecutive slot centers.
     *  Three slots evenly spaced → 120° apart. */
    private static final double SLOT_SPACING_DEG = 120.0;

    /** Maximum angular error (degrees) between the drum's measured position and
     *  a slot's center before observations are attributed to that slot.
     *  A tighter value prevents mis-attributing reads taken while the drum is
     *  still in transit. */
    private static final double SLOT_ASSIGN_TOLERANCE = 15.0;

    // -----------------------------------------------------------------------
    // Scan timing
    // -----------------------------------------------------------------------

    /** Scaling factor: estimated servo travel time (ms) per degree of rotation.
     *  Computed as {@code scanDelayMs = deltaCW * msPerDegree} and clamped to
     *  [{@link #minWait}, {@link #maxWait}].  Kept for compatibility; may not
     *  currently be used by active code paths. */
    private static final double msPerDegree = 0.6;

    /** Minimum scan delay (ms): always wait at least this long after issuing a
     *  moveTo command before reading the sensor to avoid transit-phase reads. */
    private static final double minWait = 100;

    /** Maximum scan delay (ms): caps the computed delay even for large angular
     *  moves so the robot does not stall waiting for the sensor. */
    private static final double maxWait = 300;

    /** Minimum milliseconds between consecutive unknown-slot scan attempts.
     *  Prevents the drum from spinning continuously when it is full of UNKNOWN
     *  slots and none of them can be resolved. */
    public static long UNKNOWN_SCAN_COOLDOWN_MS = 250;

    // -----------------------------------------------------------------------
    // Colour classification thresholds (fractions of total observations)
    // -----------------------------------------------------------------------

    /** Fraction of readings that must be GREEN for a slot to be declared GREEN.
     *  Lower values make green detection more sensitive (easier to trigger). */
    public static double GREEN_THRESHOLD = 0.2;

    /** Fraction of readings that must be PURPLE for a slot to be declared PURPLE. */
    public static double PURPLE_THRESHOLD = 0.2;

    /** Fraction of readings that must be EMPTY for a slot to be declared EMPTY.
     *  High threshold (0.9) prevents spurious "empty" declarations when an artifact
     *  is briefly wobbling in the slot. */
    public static double EMPTY_THRESHOLD = 0.9;

    /** Fraction of readings that must be UNKNOWN for a slot to be declared UNKNOWN. */
    public static double UNKNOWN_THRESHOLD = 0.6;

    // -----------------------------------------------------------------------
    // Observation windowing
    // -----------------------------------------------------------------------

    /** Minimum number of sensor observations accumulated before a colour decision
     *  is made for a slot.  Prevents snap decisions from single noisy reads. */
    public static int MIN_HITS_FOR_DECISION = 4;

    /** Maximum number of observations kept per slot (sliding-window cap).
     *  Older samples are proportionally scaled down by
     *  {@link org.firstinspires.ftc.teamcode.subsystems.indexerUtil.SlotObservation#trimToMax}
     *  when this limit is reached. */
    public static int MAX_HITS_TO_KEEP = 20;

    /** Angular tolerance (degrees) used by {@link #isWithinTargetDegrees} to check
     *  whether the drum has reached its commanded position.  Used by auto-advance
     *  logic to confirm the drum is settled before calling {@link #findEmptySlot}. */
    public static double ADVANCE_ANGLE_TOLERANCE = 5.0;

    // -----------------------------------------------------------------------
    // Telemetry
    // -----------------------------------------------------------------------

    /** Optional telemetry output for debug.  {@code null} means no telemetry output. */
    private Telemetry telemetry = null;

    /**
     * Attaches a telemetry object for debug output in the classification pipeline.
     * @param t the FTC telemetry instance to write to; or {@code null} to disable
     */
    public void setTelemetry(Telemetry t) { telemetry = t; }

    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------

    /** Wrapped colour sensor that classifies artifacts as GREEN, PURPLE, EMPTY, or UNKNOWN. */
    private final ColorSensorSystem colorSensor;

    /** Closed-loop position controller for the drum's continuous-rotation servo. */
    private final CRServoPositionControl servoControl;

    // -----------------------------------------------------------------------
    // Internal state
    // -----------------------------------------------------------------------

    /** The currently targeted drum slot (the slot that is (or will be) aligned
     *  with the sensor or shooter). */
    private IndexerState state = IndexerState.zero;

    /** {@code true} when the drum is oriented for intaking (sensor at intake opening);
     *  {@code false} when oriented for outtaking (shooter barrel position). */
    private boolean intaking = true;

    /** {@code true} when an autonomous outtake sequence is active.  Causes the
     *  drum to use {@link #autoOuttakeOffsetAngle} instead of the teleop offset. */
    private boolean autoOuttaking = false;

    /** {@code true} when at least one artifact is present in the drum (either
     *  detected by the sensor right now, or stored as GREEN/PURPLE in a slot's memory).
     *  Used to switch between loaded and unloaded servo gains. */
    private boolean loaded = false;

    /** {@code true} when all three slot memories are non-EMPTY (GREEN, PURPLE, or UNKNOWN).
     *  Acts as the "drum is full" flag used to stop auto-advance from looking for an empty slot. */
    private boolean noEmpty = false;

    /**
     * Returns whether all three slots in memory are occupied (no EMPTY slots stored).
     * @return {@code true} if the drum is full
     */
    public boolean isFull() { return noEmpty; }

    // -----------------------------------------------------------------------
    // Per-slot state
    // -----------------------------------------------------------------------

    /** Array of three {@link SlotState} objects, one per drum slot (index 0, 1, 2).
     *  Each holds the stored colour, hit counts, and fill-cycle flags for that slot. */
    private final SlotState[] slots = {
            new SlotState(), // slot 0 (index zero)
            new SlotState(), // slot 1 (index one)
            new SlotState()  // slot 2 (index two)
    };

    // -----------------------------------------------------------------------
    // Timers
    // -----------------------------------------------------------------------

    /** Timer used to compute the estimated scan delay after issuing a moveTo command.
     *  Compared against {@link #scanDelayMs} to know when the drum is likely on-target. */
    private final ElapsedTime scanTimer = new ElapsedTime();

    /** Estimated time (ms) for the drum to travel to the most recently commanded slot. */
    private double scanDelayMs;

    /** Cooldown timer for the unknown-slot scan logic.  Prevents the drum from
     *  spinning continuously in a failed scan loop. */
    private final ElapsedTime unknownScanTimer = new ElapsedTime();

    // -----------------------------------------------------------------------
    // Dashboard edge-detection state
    // -----------------------------------------------------------------------

    /** Previous value of {@link #dashAdvance} used to detect rising edges
     *  (false→true transitions) rather than reacting every loop while it is held true. */
    private boolean lastDashAdvance = false;

    /** Previous value of {@link #dashTargetSlot} used to detect changes so
     *  moveTo is only called once per slot selection, not every loop. */
    private int lastDashTargetSlot = -1;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Creates an Indexer instance.  Retrieves the servo and encoder from the
     * hardware map, constructs the control and sensor subsystems, and resets timers.
     *
     * @param hardwareMap the robot's hardware map used to resolve:
     *                    <ul>
     *                      <li>{@code "index"} – the drum CR servo</li>
     *                      <li>{@code "indexAnalog"} – the drum encoder analog input</li>
     *                      <li>{@code "color"} – the colour sensor (inside ColorSensorSystem)</li>
     *                    </ul>
     */
    public Indexer(HardwareMap hardwareMap) {
        CRServo servo = hardwareMap.get(CRServo.class, "index");               // fetch the drum CR servo
        AnalogInput analog = hardwareMap.get(AnalogInput.class, "indexAnalog"); // fetch the drum angle encoder

        // Build the closed-loop servo controller using the servo and its encoder.
        servoControl = new CRServoPositionControl(servo, analog);

        // Build the colour sensor wrapper (fetches the sensor internally by name).
        colorSensor = new ColorSensorSystem(hardwareMap);

        unknownScanTimer.reset(); // start the cooldown timer so the first scan isn't delayed
    }

    // -----------------------------------------------------------------------
    // Getters
    // -----------------------------------------------------------------------

    /** Returns the currently targeted drum slot. */
    public IndexerState getState() { return state; }

    /** Returns {@code true} if the drum is in intaking mode. */
    public boolean isIntaking() { return intaking; }

    /**
     * Returns the stored colour for the specified slot.
     * @param s the slot to query
     * @return the {@link ArtifactColor} stored in that slot's memory
     */
    public ArtifactColor getColorAt(IndexerState s) { return slot(s).color; }

    /** Returns {@code true} if any artifact is detected in or remembered by the drum. */
    public boolean isLoaded() { return loaded; }

    // -----------------------------------------------------------------------
    // Slot initialisation
    // -----------------------------------------------------------------------

    /**
     * Resets all slot memories to {@link ArtifactColor#EMPTY} and clears all
     * observations and fill-cycle flags.  Call before each match or test to
     * start from a clean state.
     */
    public void initializeColors() { initializeColors(ArtifactColor.EMPTY); }

    /**
     * Resets all slot memories to the given colour.  Used at the start of an
     * autonomous routine where all three slots are pre-loaded with a known colour.
     *
     * @param initialColor the colour to assign to every slot
     */
    public void initializeColors(ArtifactColor initialColor) {
        for (SlotState slot : slots) {
            slot.color = initialColor;         // assign the same color to every slot
            slot.obs.reset();                   // clear any previous sensor observations
            slot.wasEmpty = true;               // reset the transition-detection latch
            slot.fillingHits = 0;               // clear the fill-confirmation counter
            slot.fillCycleActive = false;       // deactivate any in-progress fill cycle
        }
        recomputeNoEmpty(); // update the full/not-full flag based on new colours
    }

    /**
     * Sets each slot to a specific colour.  Used when the pre-game loading order
     * is known (e.g., slot 0 = GREEN, slot 1 = PURPLE, slot 2 = PURPLE).
     *
     * @param one   colour for slot 0
     * @param two   colour for slot 1
     * @param three colour for slot 2
     */
    public void initializeColors(ArtifactColor one, ArtifactColor two, ArtifactColor three) {
        ArtifactColor[] colors = { one, two, three }; // pack into array for indexed access
        for (int i = 0; i < slots.length; i++) {
            SlotState slot = slots[i];
            slot.color = colors[i];       // assign the per-slot colour
            slot.obs.reset();              // clear observation history
            slot.wasEmpty = true;          // reset transition latch
            slot.fillingHits = 0;          // clear fill counter
            slot.fillCycleActive = false;  // deactivate fill cycle
        }
        recomputeNoEmpty(); // update the full flag
    }

    // -----------------------------------------------------------------------
    // Angle accessor
    // -----------------------------------------------------------------------

    /**
     * Returns the drum's current measured angle, wrapped into [0, 360°).
     * Reads the continuous angle from the servo controller and applies a
     * positive-modulo wrap to ensure the value is always in range.
     *
     * @return current drum angle in degrees [0, 360)
     */
    public double getMeasuredAngle() {
        return mod(servoControl.getCurrentAngle(), 360.0); // wrap continuous angle to [0, 360)
    }

    // -----------------------------------------------------------------------
    // Mode setters
    // -----------------------------------------------------------------------

    /**
     * Switches between intaking and outtaking orientation.
     * If the mode actually changes, re-issues the current slot's target angle
     * using the new offset so the servo adjusts immediately.
     *
     * @param isIntaking {@code true} to orient the drum for intaking;
     *                   {@code false} for outtaking
     */
    public void setIntaking(boolean isIntaking) {
        if (this.intaking != isIntaking) {
            this.intaking = isIntaking;
            moveTo(state, true); // force-reissue so the offset change takes effect immediately
        }
    }

    /**
     * Switches intake mode and simultaneously commands the drum to a specific slot.
     * Used at the start of a match or after a shooting sequence to set both the
     * mode and the initial slot position in one call.
     *
     * @param isIntaking    {@code true} for intaking, {@code false} for outtaking
     * @param moveToState   the slot to move to after setting the mode
     */
    public void setIntaking(boolean isIntaking, IndexerState moveToState) {
        this.intaking = isIntaking; // update mode flag
        moveTo(moveToState, true);  // forceRecommend = true ensures servo is re-commanded
    }

    /**
     * Enables or disables the autonomous-outtake angle offset.
     * When {@code true}, slot angles use {@link #autoOuttakeOffsetAngle} instead
     * of {@link #outtakeOffsetAngle}, which may differ mechanically from the
     * teleop shooting position.  Re-issues the current slot target after switching.
     *
     * @param isAutoOuttaking {@code true} to activate autonomous-outtake offset
     */
    public void setAutoOuttaking(boolean isAutoOuttaking) {
        this.autoOuttaking = isAutoOuttaking;
        moveTo(state); // recompute and reissue target with the new offset
    }

    // -----------------------------------------------------------------------
    // Colour-based movement
    // -----------------------------------------------------------------------

    /**
     * Commands the drum to rotate to the best slot for the desired colour.
     * "Best" is determined by {@link #scoreSlotForTarget}: a slot with an exact
     * match scores 100; UNKNOWN scores 80; the other non-empty colour scores 60.
     * EMPTY slots score 0 (skip them; an artifact is needed for outtaking).
     *
     * @param desired the target colour to seek
     * @return {@code true} if a suitable slot was found and the move was issued;
     *         {@code false} if no non-empty slot exists for the desired colour
     */
    public boolean moveToColor(ArtifactColor desired) {
        IndexerState target = findBestSlotForColor(desired); // find the highest-scoring slot
        if (target == null) return false; // no usable slot found; abort
        moveTo(target);                   // command the drum to rotate
        return true;
    }

    // -----------------------------------------------------------------------
    // Core moveTo methods
    // -----------------------------------------------------------------------

    /**
     * Commands the drum to rotate to the given slot.
     * If the target is already the current state, this is a no-op (no duplicate move).
     *
     * @param newState the slot to rotate to
     */
    public void moveTo(IndexerState newState) {
        moveTo(newState, false); // use default: do not force-reissue if already at target
    }

    /**
     * Commands the drum to rotate to the given slot, with optional force-reissue.
     *
     * <p>When {@code forceRecommend} is {@code true}, the target angle is always
     * sent to the servo controller even if {@code newState == state}.  This is
     * needed when the angle offset has changed (e.g., switching intake/outtake mode)
     * and the servo needs to move to a new physical position even if the logical
     * slot index hasn't changed.
     *
     * @param newState        the slot to rotate to
     * @param forceRecommend  if {@code true}, re-sends the angle even if already at target
     */
    public void moveTo(IndexerState newState, boolean forceRecommend) {
        if (!forceRecommend && newState == state) return; // already there and no force; nothing to do

        double targetAngle = getSlotCenterAngle(newState); // compute the physical angle for this slot

        // Estimate the clockwise travel distance to compute a scan delay.
        double currentWrapped = getMeasuredAngle();
        double deltaCW = targetAngle - currentWrapped;
        if (deltaCW < 0) deltaCW += 360.0; // wrap negative delta to positive clockwise arc

        // Estimate travel time and clamp to safe bounds so sensors are not read mid-transit.
        scanDelayMs = clamp(deltaCW * msPerDegree, minWait, maxWait);
        scanTimer.reset(); // start the travel-time countdown

        servoControl.clearOpenLoop();           // ensure closed-loop mode is active (not blasting)
        servoControl.moveToAngle(targetAngle);  // issue the position command to the servo controller
        state = newState;                       // update logical state immediately
    }

    // -----------------------------------------------------------------------
    // Quickspin API
    // -----------------------------------------------------------------------

    /**
     * Prepares the drum for a "quickspin" timed outtake by rotating to the
     * correct starting slot so that artifacts exit in the desired colour order.
     *
     * <p>Prerequisites:
     * <ul>
     *   <li>{@code desiredOrder} must be exactly 2 PURPLE + 1 GREEN (the only
     *       valid combination in this game's scoring motif).</li>
     *   <li>The drum must be fully loaded (2 PURPLE + 1 GREEN).</li>
     *   <li>The drum must be in intaking mode so the correct physical offset applies.</li>
     * </ul>
     *
     * @param desiredOrder the 3-element firing order to achieve;
     *                     e.g., {@code {GREEN, PURPLE, PURPLE}} means GREEN fires first
     * @return {@code true} if the drum was successfully rotated to the starting slot;
     *         {@code false} if validation failed (wrong colour mix or invalid input)
     */
    public boolean prepareQuickspin(ArtifactColor[] desiredOrder) {
        if (desiredOrder == null || desiredOrder.length != 3) return false; // input sanity check
        if (!isTwoPurpleOneGreen(desiredOrder)) return false; // must be exactly 2P + 1G

        if (!storedSlotsValidForQuickspin()) return false; // drum must contain 2P + 1G

        setIntaking(true); // quickspin must start in intaking mode for correct angle offsets

        // Search all three starting positions for one that produces the desired order.
        for (IndexerState s : IndexerState.values()) {
            int x = s.index;
            if (matchesQuickspin(x, desiredOrder)) {
                moveTo(s); // found the right starting position; rotate there
                return true;
            }
        }
        // Should never reach here if validation passed (all permutations were checked above).
        return false;
    }

    // -----------------------------------------------------------------------
    // Open-loop power control (used during timed outtake blast)
    // -----------------------------------------------------------------------

    /**
     * Commands the servo to run at a fixed open-loop power.
     * Overrides the closed-loop position controller until
     * {@link #stopIndexerPower()} is called.  Used during the "full blast"
     * outtake where the drum spins continuously at a set power for a timed duration.
     *
     * @param power servo power in [-1.0, 1.0]; positive = forward rotation
     */
    /** Open-loop blast: full power until caller stops it (or sets another power). */
    public void setIndexerPower(double power) { servoControl.setOpenLoopPower(power); }

    /**
     * Stops the open-loop blast and returns to closed-loop mode.
     * Sets power to 0 first, then clears the override flag so the
     * position controller resumes on the next {@link #update()} call.
     */
    public void stopIndexerPower() {
        servoControl.setOpenLoopPower(0); // zero the motor first
        servoControl.clearOpenLoop();     // re-enable closed-loop position control
    }

    // -----------------------------------------------------------------------
    // Main update loop
    // -----------------------------------------------------------------------

    /**
     * Must be called every robot loop iteration to keep the indexer running.
     *
     * <p>Processing order:
     * <ol>
     *   <li><b>Dashboard overrides</b> – handle manual slot advance and target-slot commands.</li>
     *   <li><b>Loaded state + servo</b> – determine if any artifact is present (sensor or memory),
     *       apply loaded/unloaded gains, run one iteration of the servo PD controller.</li>
     *   <li><b>Colour classification</b> – if intaking, scanning enabled, and not all classified,
     *       record a sensor sample for the closest slot and update its colour memory.</li>
     *   <li><b>Full flag recompute</b> – update {@link #noEmpty} based on current slot memories.</li>
     *   <li><b>Auto-advance</b> – if an empty slot exists and the policy requests it, move there.</li>
     *   <li><b>Unknown scan</b> – if the drum is full but some slots are UNKNOWN, rotate to them.</li>
     * </ol>
     */
    // update loop:
    // 1) Handle dashboard overrides
    // 2) Update loaded state and servo control
    // 3) Classify the slot under the sensor and optionally auto-advance
    // 4) Recompute FULL flag
    // 5) If intaking and any EMPTY exists, always seek an EMPTY slot (skip UNKNOWN too)
    // 6) If full and still have UNKNOWN slots, optionally move to unknown slots for rescan
    public void update() {
        handleDashboardCommands(); // step 1: process any FTC Dashboard manual overrides

        refreshLoadedAndServo(); // step 2: update sensor presence, loaded state, servo PD

        // Step 3: run colour classification only while actively intaking and scanning is on,
        // and only while at least one slot is still unclassified (avoid unnecessary work).
        if (intaking && SCAN_COLORS && !allClassified()) {
            updateSlotClassification(debugClosestSlot()); // classify the slot currently under the sensor
        }

        // Step 4: update the FULL flag based on latest slot colour memories.
        recomputeNoEmpty();

        // Step 5a: SEEK_EMPTY auto-advance – move to an empty slot every iteration if settled.
        if (AUTO_ADVANCEMENT_MODE == AutoAdvancement.SEEK_EMPTY &&
                intaking &&
                !noEmpty &&
                isWithinTargetDegrees(ADVANCE_ANGLE_TOLERANCE)) {
            moveTo(findEmptySlot()); // advance to the nearest empty slot
        }

        // Step 5b: QUICK_SEEK auto-advance – only advance when the sensor currently sees an artifact.
        if(AUTO_ADVANCEMENT_MODE == AutoAdvancement.QUICK_SEEK)
        {
            // Trigger only if: artifact currently detected AND drum has an empty slot AND drum is settled.
            if(colorSensor.hasArtifact() && !noEmpty && isWithinTargetDegrees(ADVANCE_ANGLE_TOLERANCE))
            {
                moveTo(findEmptySlot()); // move to the next available empty slot
            }
        }

        // Step 6: if drum is full but some slots are still UNKNOWN, rotate to them for a rescan.
        // A cooldown prevents spinning the drum in a tight loop if the colour can't be determined.
        if (ENABLE_FULL_UNKNOWN_SCAN &&
                intaking &&
                noEmpty &&
                unknownScanTimer.milliseconds() >= UNKNOWN_SCAN_COOLDOWN_MS) {
            IndexerState target = findNextSlotWithStoredColor(state.last(), ArtifactColor.UNKNOWN);
            if (target != null) {
                moveTo(target);               // rotate to the unclassified slot for re-read
                unknownScanTimer.reset();     // reset cooldown so we don't spam this move
            }
        }
    }

    // -----------------------------------------------------------------------
    // Private: dashboard override handling
    // -----------------------------------------------------------------------

    /**
     * Handles the FTC Dashboard advance and target-slot commands.
     * Called once per {@link #update()} iteration.
     * Uses edge detection to execute commands exactly once per dashboard change.
     */
    private void handleDashboardCommands() {
        // Rising-edge detection: only advance once when dashAdvance goes false→true.
        if (dashAdvance && !lastDashAdvance) {
            moveTo(state.next()); // advance to the next slot (wraps from 2 → 0)
        }
        lastDashAdvance = dashAdvance; // store current value for next-iteration comparison

        // Direct target-slot: only re-issue moveTo when the slot selection changes.
        if (dashTargetSlot != lastDashTargetSlot) {
            if (dashTargetSlot >= 0 && dashTargetSlot <= 2) {
                moveTo(IndexerState.values()[dashTargetSlot], true); // force-move to the selected slot
            }
            lastDashTargetSlot = dashTargetSlot; // store for next-iteration comparison
        }
    }

    // -----------------------------------------------------------------------
    // Private: loaded state and servo update
    // -----------------------------------------------------------------------

    /**
     * Determines the "loaded" flag (is any artifact present?) and runs one
     * iteration of the servo position controller.
     *
     * <p>The loaded flag is {@code true} when either:
     * <ul>
     *   <li>the colour sensor currently detects an artifact nearby, OR</li>
     *   <li>any slot memory stores GREEN or PURPLE (i.e., a confirmed artifact).</li>
     * </ul>
     * The flag is passed to {@link CRServoPositionControl#setLoaded(boolean)} to
     * switch between the heavier "loaded" PD gains and lighter "unloaded" gains.
     */
    private void refreshLoadedAndServo() {
        // Start with the live sensor reading.
        boolean hasAnyArtifact = colorSensor.hasArtifact();

        // Also check stored slot memories: if any slot remembers a real artifact, we're loaded.
        for (SlotState slot : slots) {
            if (slot.color == ArtifactColor.GREEN || slot.color == ArtifactColor.PURPLE) {
                hasAnyArtifact = true; // confirmed artifact in memory; no need to check further
                break;
            }
        }

        loaded = hasAnyArtifact;               // update cached loaded flag
        servoControl.setLoaded(loaded);        // switch servo gains based on load state
        servoControl.update();                 // run one PD control iteration
    }

    // -----------------------------------------------------------------------
    // Private: slot geometry
    // -----------------------------------------------------------------------

    /**
     * Computes the physical drum angle (degrees) that aligns the given slot
     * with the active sensing/shooting position, accounting for the current mode.
     *
     * @param s the slot whose center angle to compute
     * @return drum angle in [0, 360°) for the given slot in the current mode
     */
    private double getSlotCenterAngle(IndexerState s) {
        double angle = s.index * SLOT_SPACING_DEG; // base angle: 0°, 120°, or 240°
        angle += offsetAngle;                       // apply calibration zero-offset

        if (autoOuttaking) {
            angle += autoOuttakeOffsetAngle; // autonomous outtake uses a separate offset
        }
        else if (!intaking) {
            angle += outtakeOffsetAngle; // teleop outtake: rotate ~186° to face shooter
        }
        // In intaking mode no additional offset is applied (sensor is at 0 + offsetAngle).

        return mod(angle, 360.0); // wrap to [0, 360) so angles are unambiguous
    }

    // -----------------------------------------------------------------------
    // Colour assignment (manual override)
    // -----------------------------------------------------------------------

    /**
     * Manually assigns a colour to a specific slot and resets its observation
     * history.  Used by autonomous routines where the pre-loaded colours are known,
     * so the sensor does not need to re-classify.
     *
     * @param slotState the slot to assign
     * @param color     the colour to store in that slot's memory
     */
    public void assignSlotColor(IndexerState slotState, ArtifactColor color) {
        SlotState slot = slot(slotState);

        slot.color = color;   // set the stored colour directly, bypassing the sensor

        slot.obs.reset(); // clear sensor observations so the classifier doesn't override this assignment

        // Update the empty-tracking latch based on the new assigned colour.
        boolean isEmpty = (color == ArtifactColor.EMPTY || color == ArtifactColor.UNKNOWN);
        slot.wasEmpty = isEmpty;       // true for empty/unknown; false for green/purple
        slot.fillingHits = 0;          // clear fill counter
        slot.fillCycleActive = false;  // deactivate fill cycle

        recomputeNoEmpty(); // update the full flag with the new assignment
    }

    // -----------------------------------------------------------------------
    // Private: angle helpers
    // -----------------------------------------------------------------------

    /**
     * Computes the absolute angular error (shortest path) between two angles.
     *
     * @param a first angle in degrees
     * @param b second angle in degrees
     * @return absolute angular difference in [0, 180°]
     */
    private double angleError(double a, double b) {
        // Shift by 180°, take mod 360°, subtract 180° to fold into [-180, 180], then abs.
        return Math.abs(mod(a - b + 180.0, 360.0) - 180.0);
    }

    // -----------------------------------------------------------------------
    // Private: classification and telemetry
    // -----------------------------------------------------------------------

    /**
     * Reads the colour sensor and records one observation for each slot that is
     * currently within {@link #SLOT_ASSIGN_TOLERANCE} of the sensor position.
     * After accumulating {@link #MIN_HITS_FOR_DECISION} samples, resolves the
     * probable colour and updates the slot's memory.
     *
     * <p>The "protect known" guard prevents a good GREEN or PURPLE classification
     * from being overwritten by a transient UNKNOWN or EMPTY read (e.g., if the
     * artifact wiggles slightly and the sensor briefly loses it).
     *
     * @param currentSlot the slot that was closest to the sensor on this cycle
     *                    (used to gate telemetry output to only the active slot)
     */
    private void updateSlotClassification(IndexerState currentSlot) {
        double currentAngle = getMeasuredAngle(); // current drum angle in [0, 360)

        // Check every slot: multiple slots could theoretically be within tolerance
        // (shouldn't happen with 120° spacing and a 15° tolerance, but we check all for robustness).
        for (IndexerState s : IndexerState.values()) {
            double err = angleError(currentAngle, getSlotCenterAngle(s));
            if (err > SLOT_ASSIGN_TOLERANCE) continue; // slot is not under the sensor; skip

            SlotState slot = slot(s);

            // Read the sensor: presence check determines whether to classify or mark EMPTY.
            boolean hasArtifact = colorSensor.hasArtifact();
            ArtifactColor instantColor = hasArtifact
                    ? colorSensor.classifyColorOnly() // artifact present: determine its colour
                    : ArtifactColor.EMPTY;             // nothing there; record EMPTY

            // If this slot was previously empty and now sees something, reset observations
            // so old EMPTY counts don't dilute the new non-empty classification.
            if (slot.wasEmpty && hasArtifact) {
                slot.obs.reset(); // wipe the old observation window for a clean re-read
            }

            slot.obs.record(instantColor);         // add this sample to the observation window
            slot.obs.trimToMax(MAX_HITS_TO_KEEP);  // cap the window size to prevent stale data buildup

            int total = slot.obs.totalHits(); // total samples in the current observation window

            if (total >= MIN_HITS_FOR_DECISION) {
                // Enough data: try to resolve a colour from the accumulated observations.
                ArtifactColor candidate = slot.obs.resolveWithThreshold(
                        GREEN_THRESHOLD,
                        PURPLE_THRESHOLD,
                        EMPTY_THRESHOLD,
                        UNKNOWN_THRESHOLD
                );

                ArtifactColor currentColor = slot.color;

                // "Protect known" guard: if the slot is already classified as GREEN or PURPLE,
                // don't let a transient UNKNOWN/EMPTY candidate overwrite it.
                boolean protectKnown =
                        (candidate == ArtifactColor.UNKNOWN || candidate == ArtifactColor.EMPTY) &&
                                (currentColor == ArtifactColor.GREEN || currentColor == ArtifactColor.PURPLE);

                if (!protectKnown && candidate != currentColor) {
                    slot.color = candidate; // update the stored colour to the new classification
                }
            }

            // Update the empty-transition latch based solely on the sensor (not the stored colour).
            slot.wasEmpty = !hasArtifact;

            // Output detailed telemetry for the currently active slot only.
            if (telemetry != null && s == currentSlot) {
                int totalHitsTelemetry = slot.obs.totalHits();
                telemetry.addData("Loaded", loaded);
                telemetry.addData("Full(noEmpty)", noEmpty);
                telemetry.addData("Slot " + s + " hit %",
                        String.format(
                                "G: %.0f%%, P: %.0f%%, E: %.0f%%, U: %.0f%%",
                                totalHitsTelemetry > 0 ? slot.obs.greenHits  * 100.0 / totalHitsTelemetry : 0,
                                totalHitsTelemetry > 0 ? slot.obs.purpleHits * 100.0 / totalHitsTelemetry : 0,
                                totalHitsTelemetry > 0 ? slot.obs.emptyHits  * 100.0 / totalHitsTelemetry : 0,
                                totalHitsTelemetry > 0 ? slot.obs.unknownHits* 100.0 / totalHitsTelemetry : 0
                        ));
                colorSensor.addTelemetry(telemetry); // add full sensor HSV breakdown
            }
        }
    }

    // -----------------------------------------------------------------------
    // Debug / telemetry helpers
    // -----------------------------------------------------------------------

    /**
     * Returns the slot whose physical center angle is closest to the drum's
     * current measured angle.  Used during classification to determine which
     * slot is currently under the sensor.
     *
     * @return the nearest {@link IndexerState} to the current drum position
     */
    public IndexerState debugClosestSlot() {
        double current = getMeasuredAngle(); // current drum angle

        IndexerState best = null;
        double bestErr = Double.MAX_VALUE;

        for (IndexerState s : IndexerState.values()) {
            double err = angleError(current, getSlotCenterAngle(s));
            if (err < bestErr) {
                bestErr = err; // found a closer slot
                best = s;
            }
        }
        return best; // the slot with the smallest angular error to current position
    }

    /**
     * Returns the angular error (degrees) between the drum's current position and
     * the closest slot's center.  Useful in telemetry to assess positional accuracy.
     *
     * @return absolute angular error in [0, 180°]
     */
    public double debugClosestSlotErrorDeg() {
        IndexerState s = debugClosestSlot(); // find the closest slot first
        return angleError(getMeasuredAngle(), getSlotCenterAngle(s)); // compute error to that slot
    }

    /**
     * Returns the angular error (degrees) between the drum's current position and
     * the center angle of the specified slot.
     *
     * @param s the slot to measure against
     * @return absolute angular error in [0, 180°]
     */
    public double debugSlotErrorDeg(IndexerState s) {
        return angleError(getMeasuredAngle(), getSlotCenterAngle(s));
    }

    /**
     * Returns {@code true} if the given slot is currently within
     * {@link #SLOT_ASSIGN_TOLERANCE} of the sensor – i.e., the slot is "over" it.
     *
     * @param s the slot to test
     * @return {@code true} if the slot is aligned with the colour sensor
     */
    public boolean debugSlotIsOverSensor(IndexerState s) {
        return debugSlotErrorDeg(s) <= SLOT_ASSIGN_TOLERANCE;
    }

    /**
     * Returns a human-readable string describing the drum's current alignment status.
     * Used for telemetry dashboards during debugging.
     *
     * @return description such as {@code "Aligned with zero"} or
     *         {@code "Between slots (err=14.2 degrees)"}
     */
    public String debugAssignmentReason() {
        IndexerState s = debugClosestSlot();
        double err = debugClosestSlotErrorDeg();

        if (err > SLOT_ASSIGN_TOLERANCE) {
            return "Between slots (err=" + String.format("%.1f", err) + " degrees)";
        }
        return "Aligned with " + s;
    }

    /**
     * Returns the angular error of the second-closest slot (degrees).
     * Used to verify that the closest-slot algorithm is not confused by two slots
     * being nearly equidistant (which would indicate the drum is exactly between slots).
     *
     * @return angular error of the second-nearest slot to the current position
     */
    public double debugSecondClosestSlotErrorDeg() {
        double current = getMeasuredAngle();
        double best = Double.MAX_VALUE;
        double second = Double.MAX_VALUE;

        for (IndexerState s : IndexerState.values()) {
            double err = angleError(current, getSlotCenterAngle(s));
            if (err < best) {
                second = best; // demote the previous best to second
                best = err;
            } else if (err < second) {
                second = err; // new second-best
            }
        }
        return second;
    }

    /**
     * Returns the raw encoder voltage from the drum's analog encoder.
     * @return encoder voltage in volts
     */
    public double getVoltage() { return servoControl.getVoltage(); }

    /**
     * Returns the expected encoder voltage that corresponds to the current target angle.
     * @return target voltage in volts
     */
    public double getTargetVoltage() { return servoControl.getTargetVoltage(); }

    // -----------------------------------------------------------------------
    // Private utility methods
    // -----------------------------------------------------------------------

    /**
     * Returns the number of consecutive non-empty sensor hits required before
     * auto-advancing, based on the current {@link #AUTO_DETECT_MODE}.
     *
     * @return hit threshold for the current mode
     */
    private int requiredNonEmptyHitsToAdvance() {
        return (AUTO_DETECT_MODE == AutoDetectMode.HARD)
                ? HARD_NONEMPTY_HITS_TO_ADVANCE  // conservative: 8 hits
                : SOFT_NONEMPTY_HITS_TO_ADVANCE; // responsive: 3 hits
    }

    /**
     * Recomputes the {@link #noEmpty} flag by scanning all slot memories.
     * {@code noEmpty} is {@code true} if and only if no slot stores EMPTY.
     */
    private void recomputeNoEmpty() {
        boolean anyEmpty = false;
        for (SlotState slot : slots) {
            if (slot.color == ArtifactColor.EMPTY) {
                anyEmpty = true; // found an empty slot; short-circuit
                break;
            }
        }
        noEmpty = !anyEmpty; // full = no empty slots remain
    }

    /**
     * Returns {@code true} when every slot has been classified as either
     * GREEN or PURPLE (i.e., no UNKNOWN or EMPTY slots remain).
     * Used to skip colour scanning once all colours are known.
     *
     * @return {@code true} if all slots have a definitive colour
     */
    private boolean allClassified() {
        //checks if all are assigned a color (no unknown or empty)
        for (SlotState slot : slots) {
            if (slot.color == ArtifactColor.UNKNOWN || slot.color == ArtifactColor.EMPTY) {
                return false; // at least one slot is unresolved
            }
        }
        return true; // every slot has been assigned GREEN or PURPLE
    }

    /**
     * Searches the drum for the next slot with the given stored colour, starting
     * from {@code start} and iterating forward (by {@code next()}) up to 3 times.
     *
     * @param start the starting position (search begins at start.next())
     * @param color the colour to search for
     * @return the first matching {@link IndexerState}, or {@code null} if none found
     */
    private IndexerState findNextSlotWithStoredColor(IndexerState start, ArtifactColor color) {
        IndexerState s = start;
        for (int i = 0; i < 3; i++) {
            s = s.next(); // advance to the next slot in the ring
            if (slot(s).color == color) return s; // found a matching slot
        }
        return null; // no slot with the desired colour found after checking all three
    }

    /**
     * Finds the slot with the highest score for the desired colour using
     * {@link #scoreSlotForTarget}.  Returns the {@link IndexerState} with the
     * best match, or {@code null} if no slot scores above 0.
     *
     * @param desired the colour to search for
     * @return the best-matching slot, or {@code null} if all slots are EMPTY
     */
    public IndexerState findBestSlotForColor(ArtifactColor desired) {
        IndexerState best = null;
        int bestScore = -1;

        for (IndexerState s : IndexerState.values()) {
            ArtifactColor slotColor = slot(s).color; // read the stored colour for this slot
            int score = scoreSlotForTarget(desired, slotColor); // score this slot

            if (score > bestScore) {
                bestScore = score; // new best
                best = s;
            }
        }

        // If all slots scored 0 (all empty) there is nothing useful to fire.
        if (bestScore <= 0) return null;

        return best;
    }

    /**
     * Returns the first slot (searching forward from {@link #state}.last())
     * whose stored colour is {@link ArtifactColor#EMPTY}.
     *
     * @return the nearest empty slot, or {@code null} if none exist
     */
    public IndexerState findEmptySlot()
    {
        // Start from state.last() so we search all three slots including the current one.
        return findNextSlotWithStoredColor(state.last(), ArtifactColor.EMPTY);
    }

    /**
     * Returns an empty slot, optionally excluding the current slot.
     *
     * @param includeCurrent if {@code true}, the current slot may be returned;
     *                       if {@code false}, only the other two slots are checked
     * @return the nearest qualifying empty slot, or {@code null} if none exist
     */
    public IndexerState findEmptySlot(boolean includeCurrent)
    {
        if(includeCurrent)
            return findEmptySlot(); // search from state.last() (includes current)
        else
            return findNextSlotWithStoredColor(state.next(), ArtifactColor.EMPTY); // skip current; search forward
    }

    /**
     * Returns a priority score indicating how well a slot matches the desired colour.
     * Higher scores are more desirable for outtaking.
     *
     * <p>Scoring:
     * <ul>
     *   <li>100 – exact match (slot colour == desired)</li>
     *   <li>80  – UNKNOWN (might be the desired colour; worth trying)</li>
     *   <li>60  – the other non-empty colour (acceptable if nothing better)</li>
     *   <li>0   – EMPTY (no artifact; useless for outtaking)</li>
     * </ul>
     *
     * @param desired   the colour we want to fire
     * @param slotColor the colour stored in the candidate slot
     * @return integer score in {0, 60, 80, 100}
     */
    // gives preferences
    private int scoreSlotForTarget(ArtifactColor desired, ArtifactColor slotColor) {
        if (slotColor == desired) return 100; // perfect match; highest priority

        if (desired == ArtifactColor.GREEN) {
            if (slotColor == ArtifactColor.UNKNOWN) return 80;  // might be green; take a chance
            if (slotColor == ArtifactColor.PURPLE) return 60;   // wrong colour but non-empty; use as fallback
            if (slotColor == ArtifactColor.EMPTY) return 0;     // no artifact; skip
        }

        if (desired == ArtifactColor.PURPLE) {
            if (slotColor == ArtifactColor.UNKNOWN) return 80;  // might be purple; take a chance
            if (slotColor == ArtifactColor.GREEN) return 60;    // wrong colour but non-empty; use as fallback
            if (slotColor == ArtifactColor.EMPTY) return 0;     // no artifact; skip
        }

        return 0; // default: this slot has no value for the desired colour
    }

    /**
     * Returns {@code true} if the drum is currently within {@code toleranceDeg} degrees
     * of its commanded slot center.  Used to confirm that the servo has settled at
     * the target position before performing an operation that requires alignment.
     *
     * @param toleranceDeg maximum acceptable angular error in degrees
     * @return {@code true} if the drum is settled at the current target slot
     */
    public boolean isWithinTargetDegrees(double toleranceDeg) {
        double measured = getMeasuredAngle();          // read current drum angle
        double target = getSlotCenterAngle(state);     // compute the expected angle for the current slot
        return angleError(measured, target) <= toleranceDeg; // true if error is within tolerance
    }

    // -----------------------------------------------------------------------
    // Private math helpers
    // -----------------------------------------------------------------------

    /** Clamps {@code v} to the range [min, max]. */
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    /**
     * Positive modulo: always returns a non-negative result in [0, m).
     * Java's {@code %} can return negative values for negative operands.
     */
    private double mod(double v, double m) {
        double r = v % m;
        return r < 0 ? r + m : r; // shift negative remainder into [0, m)
    }

    /**
     * Convenience accessor: returns the {@link SlotState} for the given slot index.
     * @param s the slot whose state to retrieve
     * @return the SlotState object for slot {@code s}
     */
    private SlotState slot(IndexerState s) { return slots[s.index]; }

    // -----------------------------------------------------------------------
    // Private quickspin helpers
    // -----------------------------------------------------------------------

    /**
     * Returns {@code true} if the array contains exactly 2 PURPLE and 1 GREEN,
     * which is the only valid firing mix for a quickspin outtake in this game.
     *
     * @param arr a 3-element colour array
     * @return {@code true} if the array is valid (2P + 1G, no other colours)
     */
    //quickspin helper
    private boolean isTwoPurpleOneGreen(ArtifactColor[] arr) {
        int purple = 0;
        int green = 0;

        for (ArtifactColor c : arr) {
            if (c == ArtifactColor.PURPLE) purple++;
            else if (c == ArtifactColor.GREEN) green++;
            else return false; // any other colour (EMPTY, UNKNOWN) makes this invalid
        }
        return purple == 2 && green == 1; // must be exactly 2 purple and 1 green
    }

    /**
     * Returns {@code true} if the drum's stored slot colours are valid for a
     * quickspin (all three slots must contain exactly 2 PURPLE + 1 GREEN).
     *
     * @return {@code true} if the drum is loaded with the correct mix
     */
    private boolean storedSlotsValidForQuickspin() {
        int purple = 0;
        int green = 0;

        for (SlotState slot : slots) {
            if (slot.color == ArtifactColor.PURPLE) purple++;
            else if (slot.color == ArtifactColor.GREEN) green++;
            else return false; // EMPTY or UNKNOWN slots disqualify quickspin
        }
        return purple == 2 && green == 1; // valid only if exactly 2P + 1G are stored
    }

    /**
     * Checks whether starting a quickspin at position {@code x} would produce
     * the desired firing order.
     *
     * <p>During a forward-spin blast the drum fires slots in the order:
     * {@code (x+2)%3 → x → (x+1)%3}, so we verify that the stored colours
     * at those positions match {@code desired[0]}, {@code desired[1]}, and
     * {@code desired[2]} respectively.
     *
     * @param x       the candidate starting slot index (0, 1, or 2)
     * @param desired the 3-element desired firing order
     * @return {@code true} if starting at slot {@code x} produces the desired order
     */
    //firing order:(x + 2) % 3, x, (x + 1) % 3
    private boolean matchesQuickspin(int x, ArtifactColor[] desired) {
        return slots[(x + 2) % 3].color == desired[0] // first artifact to fire
                && slots[x].color          == desired[1] // second artifact to fire
                && slots[(x + 1) % 3].color == desired[2]; // third artifact to fire
    }

    // -----------------------------------------------------------------------
    // Public convenience check
    // -----------------------------------------------------------------------

    /**
     * Returns {@code true} if the colour sensor detects an artifact <b>and</b>
     * the drum is within 15° of the current target slot.
     *
     * <p>This combined check is used by the feedback-based intake action in
     * {@link org.firstinspires.ftc.teamcode.auto.utils.BotActions} to determine
     * when an artifact has been successfully collected: the indexer must be aligned
     * (drum settled) and the sensor must confirm presence.
     *
     * @return {@code true} if an artifact is both detected and the drum is aligned
     */
    public boolean artifactPresentAndAligned() {
        boolean hasArtifact = colorSensor.hasArtifact(); // sensor presence check
        boolean aligned = isWithinTargetDegrees(15.0);    // drum position check
        return hasArtifact && aligned; // both conditions must be true simultaneously
    }
}