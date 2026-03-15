package org.firstinspires.ftc.teamcode.subsystems.indexerUtil;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;

/**
 * CRServoPositionControl implements closed-loop angular position control for a
 * continuous-rotation (CR) servo that has an analog (potentiometer-style) absolute
 * encoder attached to its output shaft.
 *
 * <p>A CR servo does not natively accept position commands; it only responds to
 * a −1 to +1 power signal.  This class wraps the servo with:
 * <ol>
 *   <li>An <b>angle tracking</b> system that converts the encoder voltage to a
 *       continuous (unwrapped) angle, accumulating full rotations so the servo
 *       can be moved multiple revolutions without ambiguity.</li>
 *   <li>A <b>PD + static-friction compensation</b> controller that drives the
 *       servo to a commanded target angle and holds it there.</li>
 *   <li>An <b>open-loop override</b> mode used when the indexer needs to spin
 *       freely at a fixed power (e.g., during the "full blast" outtake).</li>
 *   <li>A <b>loaded/unloaded gain switching</b> feature that automatically
 *       applies different gains depending on whether artifacts are inside the
 *       indexer drum (heavier load needs stronger correction).</li>
 * </ol>
 *
 * <p>Annotated with {@link Config} so all dashboard fields are editable live
 * in the FTC Dashboard.
 */
@Config
public class CRServoPositionControl {

    // -----------------------------------------------------------------------
    // General hardware / encoder constants
    // -----------------------------------------------------------------------

    /** Maximum voltage output of the analog encoder (volts).
     *  Corresponds to one full 360° revolution.  Calibrated to 3.26 V. */
    public static double maxVoltage = 3.26;

    /** Degrees in one full encoder revolution; always 360° for a rotary encoder. */
    public static double degreesPerRev = 360.0;

    // -----------------------------------------------------------------------
    // Active (runtime) PD gains
    // -----------------------------------------------------------------------

    /** Proportional gain: error (degrees) × kP = base output power.
     *  Higher values close the loop faster but risk oscillation. */
    public static double kP = 0.004;

    /** Integral gain.  Currently 0; enable only if steady-state offset persists
     *  under load. */
    public static double kI = 0.0;

    /** Derivative gain: angular velocity (deg/s) × kD is subtracted from the
     *  output to damp overshoot.  Currently 0. */
    public static double kD = 0.0;

    /** Static friction (stiction) compensation: minimum power magnitude applied
     *  when the controller output would otherwise be too small to overcome
     *  gearbox friction and start moving the servo. */
    public static double kS = 0.07;

    // -----------------------------------------------------------------------
    // Power and stiffness limits
    // -----------------------------------------------------------------------

    /** Maximum absolute power that will be commanded to the servo.
     *  Clamps the controller output to prevent mechanical damage. */
    public static double maxPower = 1.0;

    /** Scales the proportional error term.  1.0 = normal behaviour.
     *  Can be increased for a stiffer position hold, e.g., when the indexer
     *  drum is under heavy load from the weight of three artifacts. */
    public static double stiffnessGain = 1.0; // 1.0 is normal behavior

    // -----------------------------------------------------------------------
    // Deadband
    // -----------------------------------------------------------------------

    /** Angular deadband in degrees: if |error| < deadbandDeg the servo is
     *  turned off (power = 0).  Prevents constant small oscillations while
     *  the indexer is "close enough" to its target. */
    public static double deadbandDeg = 6.0;

    /** Rotation-direction preference for {@link #moveToAngle(double)}.
     *  {@code true} = always move clockwise (positive angle increase);
     *  {@code false} = always move counter-clockwise. */
    public static boolean rotateClockwise = true;

    // -----------------------------------------------------------------------
    // Preset gain sets – switched automatically based on load state
    // -----------------------------------------------------------------------

    // Unloaded preset (no artifacts in drum) – lighter gains suffice.
    public static double unloaded_kP = 0.004;
    public static double unloaded_kI = 0.0;
    public static double unloaded_kD = 0.0;
    public static double unloaded_kS = 0.08;

    // Loaded preset (one or more artifacts in drum) – stronger integral and hold.
    public static double loaded_kP = 0.004;
    public static double loaded_kI = 0.00009;
    public static double loaded_kD = 0.0;
    public static double loaded_kS = 0.08;

    // -----------------------------------------------------------------------
    // Internal state
    // -----------------------------------------------------------------------

    /** Whether the drum currently contains at least one artifact.
     *  Used to select between loaded and unloaded gain presets. */
    private boolean loaded = false;

    /** When {@code true} the {@link #update()} method is bypassed and the servo
     *  is running on a raw power command from {@link #setOpenLoopPower(double)}.
     *  Used during the "full blast" indexer spin. */
    private boolean manualOverride = false;

    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------

    /** The continuous-rotation servo hardware object. */
    private final CRServo servo;

    /** The analog encoder reporting the servo's absolute shaft position. */
    private final AnalogInput encoder;

    // -----------------------------------------------------------------------
    // Angle tracking state
    // -----------------------------------------------------------------------

    /** Last wrapped (0–360°) encoder angle, used to detect and accumulate
     *  full-revolution wrap-arounds for continuous angle tracking. */
    private double lastWrappedDeg;

    /** Continuously-accumulated shaft angle (may exceed 360° or be negative).
     *  Monotonically increasing/decreasing across multiple revolutions so the
     *  PD target can be set without ambiguity. */
    private double continuousDeg;

    /** The target angle (continuous units, not wrapped) that the controller is
     *  driving toward. */
    private double targetDeg;

    // -----------------------------------------------------------------------
    // Derivative calculation state
    // -----------------------------------------------------------------------

    /** Shaft angle at the time of the last derivative calculation (degrees). */
    private double lastAngleDeg = 0.0;

    /** Timestamp (nanoseconds) of the last derivative calculation.
     *  Used to compute dt in seconds for the velocity calculation. */
    private long lastTimeNs = 0;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Creates a CRServoPositionControl instance.  Reads the initial encoder angle
     * and uses it as the starting point for both the continuous-angle tracker and
     * the initial target, so the servo holds its power-on position immediately.
     *
     * @param servo   the CR servo hardware object
     * @param encoder the analog encoder attached to the servo output shaft
     */
    public CRServoPositionControl(CRServo servo, AnalogInput encoder) {
        this.servo   = servo;
        this.encoder = encoder;

        // Bootstrap the continuous-angle tracker from the current raw encoder reading.
        double initial = getWrappedAngle();  // read actual shaft angle at startup
        lastWrappedDeg = initial;            // store as the "previous" wrapped angle (no delta yet)
        continuousDeg  = initial;            // initialise the continuous tracker at the current position
        targetDeg      = initial;            // hold the current position until explicitly commanded

        lastAngleDeg = continuousDeg;         // seed derivative state with current position
        lastTimeNs   = System.nanoTime();     // start the derivative timestamp
    }

    // -----------------------------------------------------------------------
    // Load state & gain switching
    // -----------------------------------------------------------------------

    /**
     * Notifies the controller whether the indexer drum is loaded with artifacts.
     * Automatically switches between the loaded and unloaded PD gain presets.
     *
     * @param hasBalls {@code true} if one or more artifacts are currently in
     *                 the drum; {@code false} when the drum is empty
     */
    // more balls is more gains
    public void setLoaded(boolean hasBalls) {
        this.loaded = hasBalls;
        if (hasBalls) {
            // Loaded: apply heavier gains to hold against artifact weight.
            kP = loaded_kP;
            kI = loaded_kI;
            kD = loaded_kD;
            kS = loaded_kS;
        } else {
            // Unloaded: revert to lighter gains for snappier, lower-current motion.
            kP = unloaded_kP;
            kI = unloaded_kI;
            kD = unloaded_kD;
            kS = unloaded_kS;
        }
    }

    /**
     * Returns whether the drum is currently in the "loaded" state.
     *
     * @return {@code true} if loaded gains are active
     */
    public boolean isLoaded() { return loaded; }

    // -----------------------------------------------------------------------
    // Open-loop override
    // -----------------------------------------------------------------------

    /**
     * Switches to open-loop mode and sets a fixed raw power on the servo.
     * The {@link #update()} method will be a no-op until {@link #clearOpenLoop()}
     * is called.  Used for the "full blast" indexer dump phase.
     *
     * @param power desired servo power in [-1.0, 1.0]
     */
    public void setOpenLoopPower(double power) {
        manualOverride = true; // disable closed-loop update
        servo.setPower(clamp(power, -1.0, 1.0)); // write clamped power directly to hardware
    }

    /**
     * Returns to closed-loop position control mode.
     * Must be called after {@link #setOpenLoopPower(double)} to re-enable
     * the PD controller in subsequent {@link #update()} calls.
     */
    public void clearOpenLoop() {
        manualOverride = false; // re-enable closed-loop update
    }

    // -----------------------------------------------------------------------
    // Main control loop
    // -----------------------------------------------------------------------

    /**
     * Runs one iteration of the position control loop.  Must be called every
     * robot loop iteration while position control is active.
     *
     * <p>If open-loop override is active ({@link #manualOverride} == true),
     * this method is a no-op and the servo continues at the last power set by
     * {@link #setOpenLoopPower(double)}.
     *
     * <p>Otherwise:
     * <ol>
     *   <li>Updates the continuous angle tracker.</li>
     *   <li>Computes the angular error.</li>
     *   <li>If within {@link #deadbandDeg}, stops the servo.</li>
     *   <li>Computes velocity and a PD output.</li>
     *   <li>Applies static-friction compensation and clamps to {@link #maxPower}.</li>
     *   <li>Writes the result to the servo.</li>
     * </ol>
     */
    public void update() {
        if (manualOverride) return; // open-loop active: skip all closed-loop logic

        updateContinuousAngle(); // integrate any wrap-arounds since the last call

        // Compute the signed angular error (continuous units).
        double error  = targetDeg - continuousDeg;
        double absErr = Math.abs(error);

        if (absErr < deadbandDeg) {
            // Within deadband: stop the servo (no restoring force needed).
            servo.setPower(0);
            // Reset derivative state so the next non-deadband motion starts clean.
            lastAngleDeg = continuousDeg;
            lastTimeNs   = System.nanoTime();
            return;
        }

        // Compute time delta for derivative calculation.
        long now   = System.nanoTime();
        double dt  = (now - lastTimeNs) * 1e-9; // convert nanoseconds to seconds
        if (dt <= 0) dt = 1e-3;                  // guard against zero/negative dt

        // Compute shaft angular velocity (degrees per second).
        double velocity = (continuousDeg - lastAngleDeg) / dt;

        // Update derivative state for next iteration.
        lastAngleDeg = continuousDeg;
        lastTimeNs   = now;

        // PD output: proportional term drives toward target; derivative damps velocity.
        // stiffnessGain scales the P term to allow a stiffer hold under load.
        double output = kP * error * stiffnessGain - kD * velocity + kI * error;

        // Static friction compensation: if the output is in the same direction as
        // the error (we need to move that way), ensure it is at least kS so the
        // servo actually starts moving despite gearbox stiction.
        if (Math.signum(output) == Math.signum(error)) {
            double sign = Math.signum(output);
            output = sign * Math.max(Math.abs(output), kS);
        }

        output = clamp(output, -maxPower, maxPower); // hard-limit to maxPower
        servo.setPower(output);                       // write computed power to hardware
    }

    // -----------------------------------------------------------------------
    // Position commands
    // -----------------------------------------------------------------------

    /**
     * Commands the servo to move to a wrapped (0–360°) target angle using
     * the preferred rotation direction ({@link #rotateClockwise}).
     *
     * <p>The wrapped angle is converted to a continuous-space target by choosing
     * the delta that matches the rotation direction preference and adding it to
     * the current continuous position.
     *
     * @param wrappedAngleDeg target angle in degrees, in the range [0, 360)
     */
    public void moveToAngle(double wrappedAngleDeg) {
        updateContinuousAngle(); // ensure continuousDeg is current before computing the delta

        // Current position in the wrapped [0, 360) space.
        double currentWrapped = mod(continuousDeg, 360.0);

        // Shortest-path delta in [-180, 180].
        double delta = wrappedAngleDeg - currentWrapped;
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        // Override delta to enforce the preferred rotation direction.
        if ( rotateClockwise && delta < 0) delta += 360;  // force clockwise (positive direction)
        if (!rotateClockwise && delta > 0) delta -= 360;  // force counter-clockwise

        targetDeg = continuousDeg + delta; // set the continuous-space target
    }

    /**
     * Moves the target by a relative amount from the current target angle.
     *
     * @param deltaDeg number of degrees to advance (positive = forward in
     *                 the current rotation direction)
     */
    public void moveBy(double deltaDeg) {
        targetDeg += deltaDeg; // simply offset the existing target
    }

    /**
     * Resets the continuous angle tracker and target to the current raw encoder
     * reading, and stops the servo.  Call this after manually repositioning the
     * servo or recovering from a fault.
     */
    public void reset() {
        double wrapped = getWrappedAngle(); // read the encoder's current physical position
        lastWrappedDeg = wrapped;           // seed the wrap-tracker
        continuousDeg  = wrapped;           // reset the continuous accumulator
        targetDeg      = wrapped;           // hold the current position (no motion)
        servo.setPower(0);                  // cut power immediately
    }

    // -----------------------------------------------------------------------
    // Internal angle tracking
    // -----------------------------------------------------------------------

    /**
     * Updates the continuous-angle tracker by comparing the current wrapped
     * encoder angle with the last recorded wrapped angle and accumulating any
     * delta (including wrap-arounds) into {@link #continuousDeg}.
     *
     * <p>Wrap-arounds are detected when the raw delta exceeds ±180°; in that
     * case 360° is added or subtracted to recover the true (small) motion delta.
     */
    // bencoder
    private void updateContinuousAngle() {
        double wrapped = getWrappedAngle();   // current raw 0–360° reading
        double delta   = wrapped - lastWrappedDeg; // naïve delta (may include wrap error)

        // Unwrap: if the sensor jumped by > 180° it crossed the 0/360 boundary.
        if (delta > 180)  delta -= 360; // e.g. 359° → 5°: delta was +354, true delta is −6
        if (delta < -180) delta += 360; // e.g. 5° → 359°: delta was −354, true delta is +6

        continuousDeg  += delta; // accumulate the true rotation delta
        lastWrappedDeg  = wrapped; // save for the next call
    }

    /**
     * Reads the raw encoder voltage, clamps it to [0, maxVoltage], and converts
     * it to a wrapped shaft angle in [0, 360°].
     *
     * @return current shaft angle in degrees, range [0, 360)
     */
    private double getWrappedAngle() {
        double v = clamp(encoder.getVoltage(), 0.0, maxVoltage); // clamp ADC reading
        return (v / maxVoltage) * degreesPerRev;                  // linear voltage → degrees
    }

    // -----------------------------------------------------------------------
    // Utility
    // -----------------------------------------------------------------------

    /** Clamps {@code v} to the range [min, max]. */
    private double clamp(double v, double min, double max) { return Math.max(min, Math.min(max, v)); }

    /**
     * Positive modulo (always returns a value in [0, m)).
     * Unlike Java's {@code %} which can return negative values, this helper
     * ensures the result is non-negative, matching mathematical modulo behaviour.
     */
    private double mod(double v, double m) {
        double r = v % m;
        return (r < 0) ? r + m : r; // shift negative remainder into [0, m)
    }

    // -----------------------------------------------------------------------
    // Debug / telemetry accessors
    // -----------------------------------------------------------------------

    /**
     * Returns the current continuous (unwrapped) shaft angle in degrees.
     * Used for telemetry and diagnostic logging.
     *
     * @return continuously-tracked shaft angle (may exceed 360° or be negative)
     */
    public double getCurrentAngle() { return continuousDeg; }

    /**
     * Returns the current continuous-space target angle in degrees.
     *
     * @return target angle in continuous space
     */
    public double getTargetAngle() { return targetDeg; }

    /**
     * Returns the target angle converted back to an encoder voltage equivalent.
     * Useful for telemetry to verify that the target position maps to a valid
     * encoder reading.
     *
     * @return expected encoder voltage for the target angle (0–maxVoltage)
     */
    public double getTargetVoltage() {
        // Wrap the continuous target back into [0, 360°].
        double wrappedDeg = targetDeg % degreesPerRev;
        if (wrappedDeg < 0) wrappedDeg += degreesPerRev; // ensure positive result

        // Convert degrees to voltage using the inverse of getWrappedAngle().
        return (wrappedDeg / degreesPerRev) * maxVoltage;
    }

    /**
     * Returns the raw encoder voltage from the analog input.
     * Used in telemetry to compare against the expected target voltage.
     *
     * @return live encoder voltage in volts
     */
    public double getVoltage() { return encoder.getVoltage(); }
}
