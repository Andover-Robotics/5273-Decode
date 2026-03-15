package org.firstinspires.ftc.teamcode.subsystems;

import static androidx.core.math.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Outtake manages the dual-flywheel shooter that launches game artifacts toward
 * the scoring goal.
 *
 * <p>Two operating modes are supported:
 * <ul>
 *   <li>{@link Mode#POWER} – open-loop: calling {@link #set(double)} directly
 *       writes a 0–1 motor power to both flywheels with no feedback.</li>
 *   <li>{@link Mode#RPM} – closed-loop PIDF: calling {@link #set(double)} sets
 *       a target RPM and {@link #periodic()} runs a PID + feedforward loop each
 *       control cycle to track it.</li>
 * </ul>
 *
 * <p>The shooter consists of two motors wired in opposite polarities (one inverted,
 * one not) so they spin in the same physical direction while facing each other.
 *
 * <p>A pre-computed range-to-RPM look-up table ({@link #REGRESSION_DATA}) along
 * with cubic polynomial ({@link #cubicRegressionRPM}) and linear-interpolation
 * helpers let the robot calculate the exact shooter speed needed for a measured
 * target distance.
 *
 * <p>Annotated with {@link Config} so all gains and tunables are editable live
 * in FTC Dashboard.
 */
@Config
public class Outtake {

    // -----------------------------------------------------------------------
    // Mode enum
    // -----------------------------------------------------------------------

    /**
     * Selects open-loop (direct power) vs closed-loop (PIDF RPM) control.
     */
    public enum Mode {
        /** Open-loop: {@link #set(double)} writes power 0–1 directly to the motors. */
        POWER,
        /** Closed-loop: {@link #set(double)} sets a target RPM tracked by PIDF each cycle. */
        RPM
    }

    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------

    /** Primary flywheel motor, mapped as {@code "outtake"}.  Inverted so both
     *  flywheels spin in the same physical direction. */
    private final MotorEx shooter;

    /** Secondary flywheel motor, mapped as {@code "outtake-2"}.  Not inverted;
     *  combined with the primary motor its shaft faces the opposite way so both
     *  contact surfaces move in the same direction against the artifact. */
    private final MotorEx shooter2;

    // -----------------------------------------------------------------------
    // Control
    // -----------------------------------------------------------------------

    /** FTCLib PID controller used in RPM mode.  Only P, I, D terms are managed
     *  here; the feedforward term {@link #f} is computed separately and added
     *  to the PID output. */
    private final PIDController controller;

    // -----------------------------------------------------------------------
    // Dashboard-tunable PIDF gains
    // -----------------------------------------------------------------------

    /** Proportional gain for the RPM PID loop.  Increase to react faster to
     *  RPM error; too high causes oscillation. */
    public static double p = 0.000567;

    /** Integral gain for the RPM PID loop.  Accumulates steady-state error over
     *  time.  Currently 0 because feedforward alone eliminates most steady-state
     *  offset. */
    public static double i = 0.0;

    /** Derivative gain for the RPM PID loop.  Damps rapid RPM changes.
     *  Currently 0; may be tuned if oscillation is observed. */
    public static double d = 0.0;

    /** Feedforward gain – approximately 1 / maxRPM, then hand-tuned.
     *  Multiplied by {@link #targetRPM} to give the approximate power needed
     *  to sustain that speed before the P term corrects any residual error. */
    public static double f = 0.0002;   // 1 / maxrpm and then tuned

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------

    /** Current operating mode ({@link Mode#POWER} or {@link Mode#RPM}). */
    public Mode mode;

    /** Last motor power written to the flywheels (0–1 in POWER mode;
     *  computed PIDF output in RPM mode). */
    private double motorPower = 0.0;

    /** Target shooter speed in RPM (used only in {@link Mode#RPM}).
     *  Defaults to 2800 RPM, which is used when no AprilTag range is available. */
    public static double targetRPM = 2800.0; // without seeing any tags

    /** Most recent measured flywheel speed in RPM, updated every call to
     *  {@link #periodic()}. */
    private double currentRPM = 0.0;

    /** Encoder ticks per motor output shaft revolution for the flywheel motors.
     *  Used to convert raw encoder velocity (ticks/s) to RPM. */
    private final double TPR = 28.0;   // encoder ticks per rotation

    // -----------------------------------------------------------------------
    // Spin-up timing
    // -----------------------------------------------------------------------

    /** Minimum time in milliseconds that the RPM must stay within tolerance
     *  before {@link #inRange(double)} reports {@code true}.  Prevents
     *  false "ready" signals from a single momentary on-target reading. */
    public static double spinupInRangeMinTime = 200; // ms

    /** Hard timeout in milliseconds for the spin-up phase.  If the flywheel
     *  has not reached the target within this window {@link #inRange(double)}
     *  returns {@code true} anyway so the robot doesn't stall indefinitely. */
    public static double spinupMaxTime = 2750; // ms

    /** Timestamp (ms) when the RPM first entered the tolerance band, or -1
     *  if it is currently outside tolerance or a new target was just set. */
    private long inRangeStartTime = -1;

    /** Timestamp (ms) when the current spin-up sequence began, or -1 if no
     *  spin-up is currently in progress. */
    private long spinupStartTime = -1;

    /** Minimum target RPM for the intake-pass-through scenario. Any target
     *  below this value is considered "stopped" for regression purposes. */
    public static double INTAKE_MIN_RPM = 3500.0;

    // -----------------------------------------------------------------------
    // Range-to-RPM look-up table (full resolution)
    // -----------------------------------------------------------------------

    /**
     * Empirically measured data pairs of {range (inches), optimal shooter RPM}.
     * Collected by shooting from each measured distance and recording the RPM
     * that produced the best scoring rate.  Used by the linear-interpolation
     * regression helpers.
     */
    private static final double[][] REGRESSION_DATA = {
            {46.4, 3400},
            {48.3, 3450},
            {50.6, 3500},
            {52.6, 3550},
            {54.6, 3575},
            {56.5, 3600},
            {58.4, 3630},
            {61.7, 3660},
            {62.4, 3685},
            {64.6, 3690},
            {66.4, 3700},
            {68.5, 3710},
            {70.5, 3720},
            {72.6, 3728},
            {74.2, 3745},
            {76.4, 3760},
            {78.7, 3775},
            {80.3, 3830},
            {82.5, 3900},
            {84.5, 3930},
            {86.4, 3985},
            {88.6, 4100},
            {92.4, 4200},
            {96.4, 4280},
            {100.6, 4370},
            {104.5, 4460},
            {108, 4540},
            {112.4, 4600},
            {116.6, 4700}
    };

    // -----------------------------------------------------------------------
    // Reduced look-up table (sparse key points only)
    // -----------------------------------------------------------------------

    /**
     * A sparse subset of {@link #REGRESSION_DATA} containing only key boundary
     * points.  Used by {@link #linearInterpolationRegressionReducedRPM(double)}
     * to provide a faster (but less accurate) interpolation when full resolution
     * is unnecessary.
     */
    private static final double[][] REGRESSION_DATA_REDUCED = {
            {46.4, 3400},
            {61.7, 3660},
            {78.7, 3775},
            {88.6, 4100},
            {116.6, 4700}
    };

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Creates an Outtake instance, initialises both flywheel motors, and
     * sets up the PID controller with the current gain values.
     *
     * @param hardwareMap the robot hardware map used to resolve motor config names
     * @param mode        initial operating mode ({@link Mode#POWER} or {@link Mode#RPM})
     */
    public Outtake(HardwareMap hardwareMap, Mode mode) {
        // Retrieve and configure the primary flywheel motor.
        // Inverted = true so that this motor spins in the correct physical direction.
        shooter = new MotorEx(hardwareMap, "outtake");
        shooter.setInverted(true);

        // Retrieve and configure the secondary flywheel motor.
        // Not inverted – when combined with the primary (facing opposite),
        // both contact surfaces push the artifact in the same direction.
        shooter2 = new MotorEx(hardwareMap, "outtake-2");
        shooter2.setInverted(false);

        this.mode = mode; // store the requested operating mode

        // Initialise the PID controller with the dashboard-tunable gains.
        controller = new PIDController(p, i, d);
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /**
     * Stops both flywheel motors immediately and resets internal state.
     * Sets {@link #motorPower} and {@link #targetRPM} to 0 so that
     * subsequent PIDF calls do not fight to restart the flywheels.
     */
    public void stop() {
        shooter.stopMotor();   // cut power to primary flywheel
        shooter2.stopMotor();  // cut power to secondary flywheel
        motorPower = 0.0;      // clear the cached power so it isn't re-applied
        targetRPM = 0.0;       // clear the RPM target so PIDF stays off
    }

    /**
     * Unified setter for both operating modes.
     *
     * <ul>
     *   <li>In {@link Mode#POWER}: {@code x} is clamped to [0, 1] and stored
     *       as the motor power applied on the next {@link #periodic()} call.</li>
     *   <li>In {@link Mode#RPM}: {@code x} is the new target RPM.  If the new
     *       target differs from the old one by more than 25 RPM, the spin-up
     *       timers are reset so the "in range" check restarts fresh.</li>
     * </ul>
     *
     * @param x desired power (POWER mode) or desired RPM (RPM mode)
     */
    public void set(double x) {
        if (mode == Mode.POWER) {
            // Clamp power to [0, 1] so the motors are never commanded backward
            // (flywheels should only spin in one direction).
            motorPower = clamp(x, 0.0, 1.0);
        } else {
            // RPM mode: if the target changes by more than 25 RPM we treat it
            // as a new spin-up request and reset the timing state.
            if (Math.abs(x - targetRPM) > 25) {
                spinupStartTime = -1;   // mark spin-up as not started
                inRangeStartTime = -1;  // mark "in range" timer as not started
            }
            targetRPM = x; // store the new target RPM for use in periodic()
        }
    }

    // -----------------------------------------------------------------------
    // Getters
    // -----------------------------------------------------------------------

    /**
     * Returns the most recently measured flywheel speed in RPM.
     * Updated each call to {@link #periodic()}.
     *
     * @return current flywheel RPM (may lag real speed by one control cycle)
     */
    public double getRPM() { return currentRPM; }

    /**
     * Returns the currently commanded target RPM.
     *
     * @return target RPM (0 when stopped)
     */
    public double getTargetRPM() { return targetRPM; }

    /**
     * Returns the last motor power written to the flywheels.
     *
     * @return motor power in the range [-1.0, 1.0] (POWER mode) or the PIDF
     *         output clamped to [-1.0, 1.0] (RPM mode)
     */
    public double getPower() { return motorPower; }

    // -----------------------------------------------------------------------
    // Control loop
    // -----------------------------------------------------------------------

    /**
     * Must be called once per robot loop iteration.  Reads the current RPM
     * from the encoder, computes PIDF output in RPM mode (or open-loop in
     * POWER mode), and writes the resulting power to both flywheel motors.
     */
    public void periodic() {
        // Compute current RPM from the primary motor's raw encoder velocity.
        // getVelocity() returns ticks/second; divide by TPR and multiply by 60
        // to get rotations per minute.
        currentRPM = shooter.getVelocity() / TPR * 60.0;

        if (mode == Mode.POWER) {
            // Open-loop path: simply write the stored motorPower to both motors.
            shooter.set(motorPower);
            shooter2.set(motorPower);
            return; // skip PIDF calculation entirely
        }

        // Sync PID gains from dashboard-editable static fields so changes take
        // effect immediately without redeploying code.
        controller.setPID(p, i, d);

        // PID term: controller compares current RPM to target and outputs a
        // correction value (positive = need to speed up, negative = overspun).
        double pid = controller.calculate(currentRPM, targetRPM);

        // Feedforward term: gives the motor a head-start power proportional to
        // the target RPM, reducing the burden on the P term.
        double ff = targetRPM * f;

        // Sum PID and feedforward for the combined PIDF output.
        motorPower = pid + ff;

        // Clamp combined output to valid motor power range.
        motorPower = clamp(motorPower, -1.0, 1.0);

        // Apply the same power to both flywheels so they stay synchronised.
        shooter.set(motorPower);
        shooter2.set(motorPower);
    }

    // -----------------------------------------------------------------------
    // Range-to-RPM regression helpers
    // -----------------------------------------------------------------------

    /**
     * Quartic polynomial regression mapping target range (inches) to shooter RPM.
     * Derived by fitting a degree-4 polynomial to the full empirical data set.
     * Kept for reference; the cubic version is currently used in production.
     *
     * @param range distance from robot to goal in inches
     * @return predicted optimal shooter RPM for that range
     */
    private double quarticRegressionRPM(double range) {
        // Evaluate: -0.000297337·r⁴ + 0.0958661·r³ - 11.09971·r² + 562.06918·r - 6981.95351
        return range * (range * (range * (range * -0.000297337 + 0.0958661) - 11.09971) + 562.06918) - 6981.95351;
    }

    /**
     * Cubic polynomial regression mapping target range (inches) to shooter RPM.
     * Fits a degree-3 polynomial to the empirical data set.
     * <b>This is the function currently used by {@link #getRegressionRPM(double)}.</b>
     *
     * @param range distance from robot to goal in inches
     * @return predicted optimal shooter RPM for that range
     */
    private double cubicRegressionRPM(double range) {
        // Evaluate: -0.000281754·r³ + 0.228245·r² - 13.14333·r + 3623.28132
        return range * (range * (range * -0.000281754 + 0.228245) - 13.14333) + 3623.28132;
    }

    /**
     * Performs piecewise linear interpolation over a given data table to map
     * range (x) to RPM (y).  Out-of-bounds ranges are clamped to the nearest
     * end segment (extrapolation via the first/last interval).
     *
     * @param range distance from robot to goal in inches
     * @param data  two-column array of {range, RPM} key points, sorted ascending
     *              by range
     * @return linearly-interpolated RPM value
     */
    private double linearInterpolation(double range, double[][] data) {
        double sum = 0;
        for (int i = 0; i < data.length - 1; i++) {
            // For the first segment, extend left to -∞; for the last, extend right to +∞.
            double min = i == 0 ? Double.MIN_VALUE : data[i][0];
            double max = i == data.length - 2 ? Double.MAX_VALUE : data[i + 1][0];
            // If range falls in this interval, compute and accumulate the interpolated value.
            if (range >= min && range < max) sum +=
                    (range - data[i][0]) / (data[i + 1][0] - data[i][0]) *   // fractional position in interval
                            (data[i + 1][1] - data[i][1]) + data[i][1];       // interpolated RPM
        }
        return sum; // only one segment fires, so sum equals that segment's result
    }

    /**
     * Linear interpolation using the full {@link #REGRESSION_DATA} table.
     *
     * @param range distance from robot to goal in inches
     * @return interpolated optimal shooter RPM
     */
    private double linearInterpolationRegressionRPM(double range) {
        return linearInterpolation(range, REGRESSION_DATA);
    }

    /**
     * Linear interpolation using the sparse {@link #REGRESSION_DATA_REDUCED} table.
     * Faster to compute than the full-table version but less precise between key points.
     *
     * @param range distance from robot to goal in inches
     * @return interpolated optimal shooter RPM (reduced accuracy)
     */
    private double linearInterpolationRegressionReducedRPM(double range) {
        return linearInterpolation(range, REGRESSION_DATA_REDUCED);
    }

    /**
     * Public facade: returns the recommended shooter RPM for a given target range.
     * Validates the range and falls back to {@link #INTAKE_MIN_RPM} for invalid
     * or zero-distance inputs.  Currently delegates to the cubic polynomial.
     *
     * @param range distance from robot to goal in inches; must be &gt; 0 and finite
     * @return optimal shooter RPM, or {@link #INTAKE_MIN_RPM} if range is invalid
     */
    public double getRegressionRPM(double range)
    {
        if (Double.isNaN(range) || range <= 0) {
            // No valid range data available (tag not visible, or called too early);
            // return a safe default RPM so the shooter is still usable.
            return INTAKE_MIN_RPM;
        }
        // Delegate to the cubic polynomial regression (best single-function fit).
        return cubicRegressionRPM(range);
    }

    // -----------------------------------------------------------------------
    // Spin-up readiness check
    // -----------------------------------------------------------------------

    /**
     * Returns {@code true} when the flywheels are considered ready to shoot.
     *
     * <p>"Ready" means either:
     * <ul>
     *   <li>The RPM has been continuously within {@code tolerance} of the target
     *       for at least {@link #spinupInRangeMinTime} ms (good enough), OR</li>
     *   <li>The spin-up has been running for longer than {@link #spinupMaxTime} ms
     *       (safety timeout – fire anyway so the robot doesn't stall).</li>
     * </ul>
     *
     * <p>After returning {@code true} all timing state is reset so the next
     * call to {@link #set(double)} starts a fresh spin-up sequence.
     *
     * @param tolerance allowable RPM deviation from {@link #targetRPM} to be
     *                  considered "in range" (e.g. 50 RPM or 100 RPM)
     * @return {@code true} if the shooter is ready to fire; {@code false} if
     *         still spinning up
     */
    public boolean inRange(double tolerance) {
        long currentTime = System.currentTimeMillis();

        // Initialise the spin-up start timestamp on the first call after a new
        // target was set (indicated by spinupStartTime == -1).
        if (spinupStartTime == -1)
        {
            spinupStartTime = currentTime;
        }

        // Check whether the measured RPM is within the specified tolerance band.
        boolean withinTolerance = Math.abs(currentRPM - targetRPM) <= tolerance;

        if (withinTolerance)
        {
            // Start the sustained-in-range timer if this is the first in-range sample.
            if (inRangeStartTime == -1)
                inRangeStartTime = currentTime;
        }
        else
        {
            // RPM left the tolerance band; reset the sustained timer.
            inRangeStartTime = -1;
        }

        // Has the RPM been continuously in range for the minimum required duration?
        boolean inRangeLongEnough = inRangeStartTime >= 0 && (currentTime - inRangeStartTime) >= spinupInRangeMinTime;

        // Has the overall spin-up exceeded the maximum allowed time (safety valve)?
        boolean spunPastMaxTime = (currentTime - spinupStartTime) >= spinupMaxTime;

        if (inRangeLongEnough || spunPastMaxTime)
        {
            // Reset timing state so the next spin-up begins clean.
            spinupStartTime = -1;
            inRangeStartTime = -1;
            return true; // shooter is ready (or timed out – fire anyway)
        }

        return false; // still spinning up; caller should keep waiting
    }
}
