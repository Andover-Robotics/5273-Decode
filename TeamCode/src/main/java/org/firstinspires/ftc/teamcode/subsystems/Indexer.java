package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Indexer {

    public enum ArtifactColor { GREEN, PURPLE, EMPTY, UNKNOWN }

    public enum IndexerState {
        zero(0), one(1), two(2);
        public final int index;
        IndexerState(int index) { this.index = index; }
        public IndexerState next() { return values()[(index + 1) % values().length]; }
    }

    // Dashboard control
    public static boolean dashAdvance = false;
    public static int dashTargetSlot = -1; // -1 = disabled; 0/1/2 = slot
    public static boolean ENABLE_AUTO_ADVANCE = false;

    // Config
    public static double offsetAngle = 17.0;
    public static double outtakeOffsetAngle = 200.0;

    // Slot spacing for color sensing
    private static final double SLOT_SPACING_DEG = 120.0;
    private static final double SLOT_ASSIGN_TOLERANCE = 15.0;

    // Scan timing (kept for compatibility if later used)
    private static final double msPerDegree = 0.6;
    private static final double minWait = 100;
    private static final double maxWait = 300;

    // Thresholds
    public static double GREEN_THRESHOLD = 0.2;
    public static double PURPLE_THRESHOLD = 0.2;
    public static double EMPTY_THRESHOLD = 0.9;
    public static double UNKNOWN_THRESHOLD = 0.6;

    // Minimum hits before allowing a color change
    public static int MIN_HITS_FOR_DECISION = 4;
    // Cap samples to keep responsiveness
    public static int MAX_HITS_TO_KEEP = 20;

    public static int NON_EMPTY_HITS_TO_ADVANCE = 5;
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
    private boolean loaded = false;

    // Per-slot state
    private final SlotState[] slots = {
            new SlotState(),
            new SlotState(),
            new SlotState()
    };

    // Scan scheduling
    private final ElapsedTime scanTimer = new ElapsedTime();
    private double scanDelayMs;

    // Dashboard edge detection
    private boolean lastDashAdvance = false;
    private int lastDashTargetSlot = -1;

    public Indexer(HardwareMap hardwareMap) {
        CRServo servo = hardwareMap.get(CRServo.class, "index");
        AnalogInput analog = hardwareMap.get(AnalogInput.class, "indexAnalog");

        servoControl = new CRServoPositionControl(servo, analog);
        colorSensor = new ColorSensorSystem(hardwareMap);
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
            slot.advanceConsumed = false;
        }
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

    /** Open-loop blast: full power until caller stops it (or sets another power). */
    public void setIndexerPower(double power) { servoControl.setOpenLoopPower(power); }
    public void stopIndexerPower() { servoControl.setOpenLoopPower(0); servoControl.clearOpenLoop(); }

    //update loop 1) Handle dashboard overrides  2) Update loaded state and servo control 3) Classify the slot under the sensor and optionally auto-advance
    public void update() {
        handleDashboardCommands();
        refreshLoadedAndServo();
        updateSlotClassification(debugClosestSlot());
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

        if (!intaking) {
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

        // Reset auto-advance gating
        slot.advanceConsumed = false;
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

            // start a fresh count when an artifact newly appears
            if (slot.wasEmpty && hasArtifact) {
                slot.obs.reset();
            }

            slot.obs.record(instantColor);
            slot.obs.trimToMax(MAX_HITS_TO_KEEP);

            int total = slot.obs.totalHits();

            // Resolve only after enough evidence
            if (total >= MIN_HITS_FOR_DECISION) {
                ArtifactColor candidate = slot.obs.resolveWithThreshold(
                        GREEN_THRESHOLD,
                        PURPLE_THRESHOLD,
                        EMPTY_THRESHOLD,
                        UNKNOWN_THRESHOLD
                );

                ArtifactColor currentColor = slot.color;

                // Prevent UNKNOWN/EMPTY from overwriting a known color
                boolean protectKnown = (candidate == ArtifactColor.UNKNOWN || candidate == ArtifactColor.EMPTY) &&
                        (currentColor == ArtifactColor.GREEN || currentColor == ArtifactColor.PURPLE);

                if (!protectKnown && candidate != currentColor) {
                    slot.color = candidate;
                }
            }

            // Auto-advance (guarded by toggle)
            if (ENABLE_AUTO_ADVANCE && s == state && intaking && isWithinTargetDegrees(ADVANCE_ANGLE_TOLERANCE)) {
                boolean slotAlreadyFull = (slot.color == ArtifactColor.GREEN || slot.color == ArtifactColor.PURPLE);
                if (!slotAlreadyFull && slot.wasEmpty && hasArtifact && !slot.advanceConsumed) {
                    moveTo(state.next());
                    slot.advanceConsumed = true;
                }
            }

            // Update empty memory / advance gating
            slot.wasEmpty = !hasArtifact;
            if (slot.wasEmpty) {
                slot.advanceConsumed = false; // allow next fill to advance
            }

            // telemetry for current slot
            if (telemetry != null && s == currentSlot) {
                int totalHits = slot.obs.totalHits();
                telemetry.addData("Loaded", loaded);
                telemetry.addData("Slot " + s + " hit %",
                        String.format(
                                "G: %.0f%%, P: %.0f%%, E: %.0f%%, U: %.0f%%",
                                totalHits > 0 ? slot.obs.greenHits * 100.0 / totalHits : 0,
                                totalHits > 0 ? slot.obs.purpleHits * 100.0 / totalHits : 0,
                                totalHits > 0 ? slot.obs.emptyHits * 100.0 / totalHits : 0,
                                totalHits > 0 ? slot.obs.unknownHits * 100.0 / totalHits : 0
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
        boolean advanceConsumed = false;
    }

    private static class SlotObservation {
        int greenHits = 0;
        int purpleHits = 0;
        int emptyHits = 0;
        int unknownHits = 0;

        void reset() { greenHits = purpleHits = emptyHits = unknownHits = 0; }

        void record(ArtifactColor c) {
            switch (c) {
                case GREEN -> greenHits++;
                case PURPLE -> purpleHits++;
                case EMPTY -> emptyHits++;
                case UNKNOWN -> unknownHits++;
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
}