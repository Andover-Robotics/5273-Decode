package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.indexerUtil.CRServoPositionControl;
import org.firstinspires.ftc.teamcode.subsystems.indexerUtil.ColorSensorSystem;

@Config
public class Indexer {

    public enum ArtifactColor { GREEN, PURPLE, EMPTY, UNKNOWN }

    public enum IndexerState {
        zero(0), one(1), two(2);
        public final int index;
        IndexerState(int index) { this.index = index; }
        public IndexerState next() { return values()[(index + 1) % values().length]; }
        public IndexerState last() { return values()[(index + values().length - 1) % values().length]; }
    }

    // Dashboard control
    public static boolean dashAdvance = false;
    public static int dashTargetSlot = -1; // -1 = disabled; 0/1/2 = slot

    //Things that should be toggled depending on use case (more robust version coming)
   // public  boolean ENABLE_AUTO_ADVANCE = true;

    public enum AutoAdvancement
    {
        DISABLED,
        SEEK_EMPTY,
        QUICK_SEEK
    }

    public AutoAdvancement AUTO_ADVANCEMENT_MODE = AutoAdvancement.QUICK_SEEK;

    public boolean ENABLE_FULL_UNKNOWN_SCAN = true;
    public boolean SCAN_COLORS = true;

    // Auto-advance detection mode
    public enum AutoDetectMode { HARD, SOFT }
    public static AutoDetectMode AUTO_DETECT_MODE = AutoDetectMode.SOFT;
    public static int HARD_NONEMPTY_HITS_TO_ADVANCE = 8;
    public static int SOFT_NONEMPTY_HITS_TO_ADVANCE = 3;


    // Offsets
    public static double offsetAngle = 77.0; // 77 for normal, 105 is for auto
    public static double autoOuttakeOffsetAngle = 32.0;
    public static double outtakeOffsetAngle = 200.0;

    // Slot spacing for color sensing
    private static final double SLOT_SPACING_DEG = 120.0;
    private static final double SLOT_ASSIGN_TOLERANCE = 15.0;

    // Scan timing (kept for compatibility if later used)
    private static final double msPerDegree = 0.6;
    private static final double minWait = 100;
    private static final double maxWait = 300;
    // Cooldown to avoid spamming moves while full+unknown
    public static long UNKNOWN_SCAN_COOLDOWN_MS = 250;

    // Thresholds
    public static double GREEN_THRESHOLD = 0.2;
    public static double PURPLE_THRESHOLD = 0.2;
    public static double EMPTY_THRESHOLD = 0.9;
    public static double UNKNOWN_THRESHOLD = 0.6;

    // Minimum hits before allowing a color change
    public static int MIN_HITS_FOR_DECISION = 4;
    // Cap samples to keep responsiveness
    public static int MAX_HITS_TO_KEEP = 20;

    public static double ADVANCE_ANGLE_TOLERANCE = 5.0;

    // Telemetry object
    private Telemetry telemetry = null;
    public void setTelemetry(Telemetry t) { telemetry = t; }

    // Objects
    private final ColorSensorSystem colorSensor;
    private final CRServoPositionControl servoControl;

    // Internal state
    private IndexerState state = IndexerState.zero;
    private boolean intaking = true;
    private boolean autoOuttaking = false;
    private boolean loaded = false;   // "anything present" (sensor OR stored memory)
    private boolean noEmpty = false;  // FULL: true when there are NO stored EMPTY slots

    public boolean isFull() { return noEmpty; }

    // Per-slot state
    private final SlotState[] slots = {
            new SlotState(),
            new SlotState(),
            new SlotState()
    };

    // Scan scheduling
    private final ElapsedTime scanTimer = new ElapsedTime();
    private double scanDelayMs;

    // Full+unknown scan cooldown
    private final ElapsedTime unknownScanTimer = new ElapsedTime();

    // Dashboard edge detection
    private boolean lastDashAdvance = false;
    private int lastDashTargetSlot = -1;

    public Indexer(HardwareMap hardwareMap) {
        CRServo servo = hardwareMap.get(CRServo.class, "index");
        AnalogInput analog = hardwareMap.get(AnalogInput.class, "indexAnalog");

        servoControl = new CRServoPositionControl(servo, analog);
        colorSensor = new ColorSensorSystem(hardwareMap);

        unknownScanTimer.reset();
    }

    // getters
    public IndexerState getState() { return state; }
    public boolean isIntaking() { return intaking; }
    public ArtifactColor getColorAt(IndexerState s) { return slot(s).color; }
    public boolean isLoaded() { return loaded; }

    /** Initialize all slots to the given color (default EMPTY) and reset observations/flags. */
    public void initializeColors() { initializeColors(ArtifactColor.EMPTY); }

    public void initializeColors(ArtifactColor initialColor) {
        for (SlotState slot : slots) {
            slot.color = initialColor;
            slot.obs.reset();
            slot.wasEmpty = true;
            slot.fillingHits = 0;
            slot.autoAdvanceArmed = false;
            slot.fillCycleActive = false;
        }
        recomputeNoEmpty();
    }

    public void initializeColors(ArtifactColor one, ArtifactColor two, ArtifactColor three) {
        ArtifactColor[] colors = { one, two, three };
        for (int i = 0; i < slots.length; i++) {
            SlotState slot = slots[i];
            slot.color = colors[i];
            slot.obs.reset();
            slot.wasEmpty = true;
            slot.fillingHits = 0;
            slot.autoAdvanceArmed = false;
            slot.fillCycleActive = false;
        }
        recomputeNoEmpty();
    }

    public double getMeasuredAngle() {
        return mod(servoControl.getCurrentAngle(), 360.0);
    }

    // api
    public void setIntaking(boolean isIntaking) {
        if (this.intaking != isIntaking) {
            this.intaking = isIntaking;
            moveTo(state, true);
        }
    }

    public void setIntaking(boolean isIntaking, IndexerState moveToState) {
        this.intaking = isIntaking;
        moveTo(moveToState, true);
    }

    public void setAutoOuttaking(boolean isAutoOuttaking) {
        this.autoOuttaking = isAutoOuttaking;
        moveTo(state);
    }

    public boolean moveToColor(ArtifactColor desired) {
        IndexerState target = findBestSlotForColor(desired);
        if (target == null) return false;
        moveTo(target);
        return true;
    }

    public void moveTo(IndexerState newState) {
        moveTo(newState, false);
    }

    // Move to a slot and forceRecommand will reissue the target even if it's the current slot.
    public void moveTo(IndexerState newState, boolean forceRecommand) {
        if (!forceRecommand && newState == state) return;

        double targetAngle = getSlotCenterAngle(newState);

        double currentWrapped = getMeasuredAngle();
        double deltaCW = targetAngle - currentWrapped;
        if (deltaCW < 0) deltaCW += 360.0;

        scanDelayMs = clamp(deltaCW * msPerDegree, minWait, maxWait);
        scanTimer.reset();

        servoControl.clearOpenLoop();
        servoControl.moveToAngle(targetAngle);
        state = newState;
    }

    //sortedquickspin api
    public boolean prepareQuickspin(ArtifactColor[] desiredOrder) {
        // Validate input
        if (desiredOrder == null || desiredOrder.length != 3) return false;
        if (!isTwoPurpleOneGreen(desiredOrder)) return false;

        // Validate stored slots
        if (!storedSlotsValidForQuickspin()) return false;

        // Quickspin must start in intaking mode
        setIntaking(true);

        // Find correct starting slot
        for (IndexerState s : IndexerState.values()) {
            int x = s.index;
            if (matchesQuickspin(x, desiredOrder)) {
                moveTo(s);
                return true;
            }
        }
        // Should never happen if validation passed
        return false;
    }

    /** Open-loop blast: full power until caller stops it (or sets another power). */
    public void setIndexerPower(double power) { servoControl.setOpenLoopPower(power); }
    public void stopIndexerPower() { servoControl.setOpenLoopPower(0); servoControl.clearOpenLoop(); }

    // update loop:
    // 1) Handle dashboard overrides
    // 2) Update loaded state and servo control
    // 3) Classify the slot under the sensor and optionally auto-advance
    // 4) Recompute FULL flag
    // 5) If intaking and any EMPTY exists, always seek an EMPTY slot (skip UNKNOWN too)
    // 6) If full and still have UNKNOWN slots, optionally move to unknown slots for rescan
    public void update() {
        handleDashboardCommands();
        refreshLoadedAndServo();

        if (intaking && SCAN_COLORS) {
            updateSlotClassification(debugClosestSlot());
        }

        // Update full flag every loop based on stored memory
        recomputeNoEmpty();

        if (AUTO_ADVANCEMENT_MODE == AutoAdvancement.SEEK_EMPTY &&
                intaking &&
                !noEmpty &&
                isWithinTargetDegrees(ADVANCE_ANGLE_TOLERANCE)) {
            moveTo(findEmptySlot());
        }
        if(AUTO_ADVANCEMENT_MODE == AutoAdvancement.QUICK_SEEK)
        {
            if(colorSensor.hasArtifact() && !noEmpty && isWithinTargetDegrees(ADVANCE_ANGLE_TOLERANCE))
            {
                moveTo(findEmptySlot());
            }
        }
        // If full and still have UNKNOWN slots, go look at them (intaking only)
        if (ENABLE_FULL_UNKNOWN_SCAN &&
                intaking &&
                noEmpty &&
                unknownScanTimer.milliseconds() >= UNKNOWN_SCAN_COOLDOWN_MS) {
            IndexerState target = findNextSlotWithStoredColor(state.last(), ArtifactColor.UNKNOWN);
            if (target != null) {
                moveTo(target);
                unknownScanTimer.reset();
            }
        }
    }

    private void handleDashboardCommands() {
        if (dashAdvance && !lastDashAdvance) {
            moveTo(state.next());
        }
        lastDashAdvance = dashAdvance;

        if (dashTargetSlot != lastDashTargetSlot) {
            if (dashTargetSlot >= 0 && dashTargetSlot <= 2) {
                moveTo(IndexerState.values()[dashTargetSlot], true);
            }
            lastDashTargetSlot = dashTargetSlot;
        }
    }

    private void refreshLoadedAndServo() {
        // Determine loaded state BEFORE control update
        boolean hasAnyArtifact = colorSensor.hasArtifact();
        for (SlotState slot : slots) {
            if (slot.color == ArtifactColor.GREEN || slot.color == ArtifactColor.PURPLE) {
                hasAnyArtifact = true;
                break;
            }
        }
        loaded = hasAnyArtifact;
        servoControl.setLoaded(loaded);
        servoControl.update();
    }

    // slot geometry
    private double getSlotCenterAngle(IndexerState s) {
        double angle = s.index * SLOT_SPACING_DEG;
        angle += offsetAngle;

        if (autoOuttaking) {
            angle += autoOuttakeOffsetAngle; // NOT 180 unless confirmed mechanically
        }
        else if (!intaking) {
            angle += outtakeOffsetAngle; // NOT 180 unless confirmed mechanically
        }

        return mod(angle, 360.0);
    }

    public void assignSlotColor(IndexerState slotState, ArtifactColor color) {
        SlotState slot = slot(slotState);

        // Assign color
        slot.color = color;

        // Reset observations so classifier doesn't fight this
        slot.obs.reset();

        // Update empty tracking
        boolean isEmpty = (color == ArtifactColor.EMPTY || color == ArtifactColor.UNKNOWN);
        slot.wasEmpty = isEmpty;
        slot.fillingHits = 0;
        slot.autoAdvanceArmed = false;
        slot.fillCycleActive = false;

        recomputeNoEmpty();
    }

    private double angleError(double a, double b) {
        return Math.abs(mod(a - b + 180.0, 360.0) - 180.0);
    }

    // classification + auto-advance
    private void updateSlotClassification(IndexerState currentSlot) {
        double currentAngle = getMeasuredAngle();

        for (IndexerState s : IndexerState.values()) {
            double err = angleError(currentAngle, getSlotCenterAngle(s));
            if (err > SLOT_ASSIGN_TOLERANCE) continue; // not over sensor

            SlotState slot = slot(s);

            boolean hasArtifact = colorSensor.hasArtifact();
            ArtifactColor instantColor = hasArtifact
                    ? colorSensor.classifyColorOnly()
                    : ArtifactColor.EMPTY;

            // If transitioning from empty->nonempty (sensor), reset observation window for faster settle
            if (slot.wasEmpty && hasArtifact) {
                slot.obs.reset();
            }

            slot.obs.record(instantColor);
            slot.obs.trimToMax(MAX_HITS_TO_KEEP);

            int total = slot.obs.totalHits();

            if (total >= MIN_HITS_FOR_DECISION) {
                ArtifactColor candidate = slot.obs.resolveWithThreshold(
                        GREEN_THRESHOLD,
                        PURPLE_THRESHOLD,
                        EMPTY_THRESHOLD,
                        UNKNOWN_THRESHOLD
                );

                ArtifactColor currentColor = slot.color;

                boolean protectKnown =
                        (candidate == ArtifactColor.UNKNOWN || candidate == ArtifactColor.EMPTY) &&
                                (currentColor == ArtifactColor.GREEN || currentColor == ArtifactColor.PURPLE);

                if (!protectKnown && candidate != currentColor) {
                    slot.color = candidate;
                }
            }

            // Update memory ONLY
            slot.wasEmpty = !hasArtifact;

            if (telemetry != null && s == currentSlot) {
                int totalHitsTelemetry = slot.obs.totalHits();
                telemetry.addData("Loaded", loaded);
                telemetry.addData("Full(noEmpty)", noEmpty);
                telemetry.addData("Slot " + s + " hit %",
                        String.format(
                                "G: %.0f%%, P: %.0f%%, E: %.0f%%, U: %.0f%%",
                                totalHitsTelemetry > 0 ? slot.obs.greenHits * 100.0 / totalHitsTelemetry : 0,
                                totalHitsTelemetry > 0 ? slot.obs.purpleHits * 100.0 / totalHitsTelemetry : 0,
                                totalHitsTelemetry > 0 ? slot.obs.emptyHits * 100.0 / totalHitsTelemetry : 0,
                                totalHitsTelemetry > 0 ? slot.obs.unknownHits * 100.0 / totalHitsTelemetry : 0
                        ));
                colorSensor.addTelemetry(telemetry);
            }
        }
    }

    // debug
    public IndexerState debugClosestSlot() {
        double current = getMeasuredAngle();

        IndexerState best = null;
        double bestErr = Double.MAX_VALUE;

        for (IndexerState s : IndexerState.values()) {
            double err = angleError(current, getSlotCenterAngle(s));
            if (err < bestErr) {
                bestErr = err;
                best = s;
            }
        }
        return best;
    }

    public double debugClosestSlotErrorDeg() {
        IndexerState s = debugClosestSlot();
        return angleError(getMeasuredAngle(), getSlotCenterAngle(s));
    }

    public double debugSlotErrorDeg(IndexerState s) {
        return angleError(getMeasuredAngle(), getSlotCenterAngle(s));
    }

    public boolean debugSlotIsOverSensor(IndexerState s) {
        return debugSlotErrorDeg(s) <= SLOT_ASSIGN_TOLERANCE;
    }

    public String debugAssignmentReason() {
        IndexerState s = debugClosestSlot();
        double err = debugClosestSlotErrorDeg();

        if (err > SLOT_ASSIGN_TOLERANCE) {
            return "Between slots (err=" + String.format("%.1f", err) + " degrees)";
        }
        return "Aligned with " + s;
    }

    public double debugSecondClosestSlotErrorDeg() {
        double current = getMeasuredAngle();
        double best = Double.MAX_VALUE;
        double second = Double.MAX_VALUE;

        for (IndexerState s : IndexerState.values()) {
            double err = angleError(current, getSlotCenterAngle(s));
            if (err < best) {
                second = best;
                best = err;
            } else if (err < second) {
                second = err;
            }
        }
        return second;
    }

    public double getVoltage() { return servoControl.getVoltage(); }
    public double getTargetVoltage() { return servoControl.getTargetVoltage(); }

    /* =========================
       UTIL
       ========================= */

    private int requiredNonEmptyHitsToAdvance() {
        return (AUTO_DETECT_MODE == AutoDetectMode.HARD)
                ? HARD_NONEMPTY_HITS_TO_ADVANCE
                : SOFT_NONEMPTY_HITS_TO_ADVANCE;
    }

    private void recomputeNoEmpty() {
        boolean anyEmpty = false;
        for (SlotState slot : slots) {
            if (slot.color == ArtifactColor.EMPTY) {
                anyEmpty = true;
                break;
            }
        }
        noEmpty = !anyEmpty;
    }

    private boolean anyUnknownStored() {
        for (SlotState slot : slots) {
            if (slot.color == ArtifactColor.UNKNOWN) return true;
        }
        return false;
    }

    private IndexerState findNextSlotWithStoredColor(IndexerState start, ArtifactColor color) {
        IndexerState s = start;
        for (int i = 0; i < 3; i++) {
            s = s.next();
            if (slot(s).color == color) return s;
        }
        return null;
    }

    public IndexerState findBestSlotForColor(ArtifactColor desired) {
        IndexerState best = null;
        int bestScore = -1;

        for (IndexerState s : IndexerState.values()) {
            ArtifactColor slotColor = slot(s).color; // use live slot color
            int score = scoreSlotForTarget(desired, slotColor);

            if (score > bestScore) {
                bestScore = score;
                best = s;
            }
        }

        // If all slots are EMPTY or useless, don't move
        if (bestScore <= 0) return null;

        return best;
    }

    public IndexerState findEmptySlot()
    {
        return findNextSlotWithStoredColor(state.last(), ArtifactColor.EMPTY);
    }

    public IndexerState findEmptySlot(boolean includeCurrent)
    {
        if(includeCurrent)
            return findEmptySlot();
        else
            return findNextSlotWithStoredColor(state.next(), ArtifactColor.EMPTY);
    }

    // gives preferences
    private int scoreSlotForTarget(ArtifactColor desired, ArtifactColor slotColor) {
        if (slotColor == desired) return 100;

        if (desired == ArtifactColor.GREEN) {
            if (slotColor == ArtifactColor.UNKNOWN) return 80;
            if (slotColor == ArtifactColor.PURPLE) return 60;
            if (slotColor == ArtifactColor.EMPTY) return 0;
        }

        if (desired == ArtifactColor.PURPLE) {
            if (slotColor == ArtifactColor.UNKNOWN) return 80;
            if (slotColor == ArtifactColor.GREEN) return 60;
            if (slotColor == ArtifactColor.EMPTY) return 0;
        }

        return 0;
    }

    public boolean isWithinTargetDegrees(double toleranceDeg) {
        double measured = getMeasuredAngle();
        double target = getSlotCenterAngle(state);
        return angleError(measured, target) <= toleranceDeg;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

    private double mod(double v, double m) {
        double r = v % m;
        return r < 0 ? r + m : r;
    }

    private SlotState slot(IndexerState s) { return slots[s.index]; }

    private static class SlotState {
        ArtifactColor color = ArtifactColor.UNKNOWN;
        SlotObservation obs = new SlotObservation();
        boolean wasEmpty = true;

        int fillingHits = 0;          // counts sensorNonEmpty hits during a fill cycle
        boolean autoAdvanceArmed = false; // kept for compatibility; no longer required
        boolean fillCycleActive = false;  // latched after stored EMPTY + sensor nonempty
    }

    private static class SlotObservation {
        int greenHits = 0;
        int purpleHits = 0;
        int emptyHits = 0;
        int unknownHits = 0;

        void reset() { greenHits = purpleHits = emptyHits = unknownHits = 0; }

        void record(ArtifactColor c) {
            switch (c) {
                case GREEN:
                    greenHits++;
                    break;
                case PURPLE:
                    purpleHits++;
                    break;
                case EMPTY:
                    emptyHits++;
                    break;
                case UNKNOWN:
                    unknownHits++;
                    break;
            }
        }

        int totalHits() { return greenHits + purpleHits + emptyHits + unknownHits; }

        void trimToMax(int maxTotal) {
            int total = totalHits();
            if (total <= maxTotal) return;
            double scale = maxTotal / (double) total;
            greenHits = (int) Math.round(greenHits * scale);
            purpleHits = (int) Math.round(purpleHits * scale);
            emptyHits = (int) Math.round(emptyHits * scale);
            unknownHits = (int) Math.round(unknownHits * scale);
        }

        ArtifactColor resolveWithThreshold(double greenThresh, double purpleThresh, double emptyThresh, double unknownThresh) {
            int total = totalHits();
            if (total == 0) return ArtifactColor.EMPTY;

            if (greenHits / (double) total >= greenThresh) return ArtifactColor.GREEN;
            if (purpleHits / (double) total >= purpleThresh) return ArtifactColor.PURPLE;
            if (emptyHits / (double) total >= emptyThresh) return ArtifactColor.EMPTY;
            if (unknownHits / (double) total >= unknownThresh) return ArtifactColor.UNKNOWN;

            return ArtifactColor.UNKNOWN; // fallback if nothing crosses threshold
        }
    }

    //quickspin helper
    private boolean isTwoPurpleOneGreen(ArtifactColor[] arr) {
        int purple = 0;
        int green = 0;

        for (ArtifactColor c : arr) {
            if (c == ArtifactColor.PURPLE) purple++;
            else if (c == ArtifactColor.GREEN) green++;
            else return false;
        }
        return purple == 2 && green == 1;
    }

    private boolean storedSlotsValidForQuickspin() {
        int purple = 0;
        int green = 0;

        for (SlotState slot : slots) {
            if (slot.color == ArtifactColor.PURPLE) purple++;
            else if (slot.color == ArtifactColor.GREEN) green++;
            else return false;
        }
        return purple == 2 && green == 1;
    }

    //firing order:(x + 2) % 3, x, (x + 1) % 3
    private boolean matchesQuickspin(int x, ArtifactColor[] desired) {
        return slots[(x + 2) % 3].color == desired[0]
                && slots[x].color == desired[1]
                && slots[(x + 1) % 3].color == desired[2];
    }

    public boolean artifactPresentAndAligned() {
        boolean hasArtifact = colorSensor.hasArtifact();
        boolean aligned = isWithinTargetDegrees(15.0);
        return hasArtifact && aligned;
    }
}