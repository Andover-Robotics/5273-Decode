package org.firstinspires.ftc.teamcode.subsystems.indexerUtil;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * BetterCRControl is a PID position controller for a continuous-rotation servo
 * equipped with an analog (potentiometer-style) encoder.
 *
 * <p>Unlike a standard servo that accepts absolute position commands,
 * a continuous-rotation (CR) servo must be driven by open-loop power signals.
 * This class closes the position loop manually:
 * <ol>
 *   <li>Read the encoder voltage and convert it to an angle (0–360°).</li>
 *   <li>Compute angular error between the target and current angle.</li>
 *   <li>Run a PID calculation to determine a corrective power.</li>
 *   <li>Apply a hold power when within the deadband to resist disturbances.</li>
 * </ol>
 *
 * <p><b>Note:</b> This class is an older/alternative implementation.
 * {@link CRServoPositionControl} is currently used in production.
 *
 * <p>Annotated with {@link Config} so all gains and thresholds can be tuned
 * live from the FTC Dashboard.
 */
@Config
public class BetterCRControl {

    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------

    /** The continuous-rotation servo being controlled. */
    private final CRServo crServo;

    /** The analog encoder attached to the servo shaft.
     *  Provides a 0–3.2 V signal proportional to the current angle. */
    private final AnalogInput encoder;

    // -----------------------------------------------------------------------
    // Dashboard-tunable PID gains
    // -----------------------------------------------------------------------

    /** Proportional gain.  Scales the error (degrees) directly into power.
     *  Higher values give faster response but increase overshoot risk. */
    public static double kp = 0.45;

    /** Integral gain.  Accumulates error over time to eliminate steady-state
     *  offset.  Currently 0; enable only if there is noticeable steady-state
     *  positional drift. */
    public static double ki = 0.0;

    /** Derivative gain.  Damps rapid changes in error to reduce overshoot.
     *  A value of 0.04 provides light damping. */
    public static double kd = 0.04;

    // -----------------------------------------------------------------------
    // Dashboard-tunable thresholds
    // -----------------------------------------------------------------------

    /** Angular error (degrees) below which the servo is considered "on target".
     *  Within this band a small hold power is applied instead of running
     *  the full PID loop, preventing high-frequency chatter. */
    public static double deadband = 3;    // degrees

    /** Minimum absolute power applied when the error exceeds the deadband.
     *  Overcomes static friction (stiction) in the servo gearbox. */
    public static double minPower = 0.08;

    /** Power applied while within the deadband to resist external disturbances
     *  (e.g., field elements pressing against the indexer).  Applied in the
     *  direction of the last non-zero error to act as a light restoring force. */
    public static double holdPower = 0.05;

    // -----------------------------------------------------------------------
    // Internal PID state
    // -----------------------------------------------------------------------

    /** Accumulated error × dt (integral term numerator). */
    private double integral = 0.0;

    /** Error from the previous loop iteration; used to compute the derivative. */
    private double lastError = 0.0;

    /** Smoothed / filtered voltage reading from the encoder.
     *  Uses an exponential moving average to reduce noise. */
    private double filteredVoltage = 0;

    /** Elapsed-time timer used to measure the dt between consecutive
     *  {@link #moveToAngle(double)} calls for the integral and derivative terms. */
    private ElapsedTime timer = new ElapsedTime();

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Creates a BetterCRControl instance and resets the delta-time timer.
     *
     * @param servo   the continuous-rotation servo to control
     * @param encoder the analog encoder attached to the servo output shaft
     */
    public BetterCRControl(CRServo servo, AnalogInput encoder) {
        this.crServo = servo;   // store servo reference
        this.encoder = encoder; // store encoder reference
        timer.reset();          // start the dt timer from zero
    }

    // -----------------------------------------------------------------------
    // Internal helpers
    // -----------------------------------------------------------------------

    /**
     * Returns a smoothed encoder voltage using a fixed-coefficient exponential
     * moving average (EMA):  {@code new = 0.6 × old + 0.4 × raw}.
     * This reduces high-frequency ADC noise without introducing significant
     * latency into the position feedback.
     *
     * @return filtered encoder voltage in volts
     */
    private double getFilteredVoltage() {
        // Blend the previous filtered value (60 %) with the new raw reading (40 %).
        filteredVoltage = 0.6 * filteredVoltage + 0.4 * encoder.getVoltage();
        return filteredVoltage;
    }

    /**
     * Converts the analog encoder voltage to a shaft angle in degrees.
     *
     * <p>The encoder produces a 0–3.2 V signal over one full 360° revolution,
     * so {@code angle = (voltage / 3.2) × 360}.
     *
     * @param v encoder voltage in volts (expected range: 0–3.2)
     * @return corresponding shaft angle in degrees (0–360)
     */
    private double voltageToAngle(double v) {
        return (v / 3.2) * 360.0; // linear mapping: 3.2 V → 360°, 0 V → 0°
    }

    /**
     * Wraps an angle into the range (-180, 180] so that the shortest-path error
     * direction is always chosen.
     *
     * @param angle raw angle or error in degrees
     * @return equivalent angle in (-180, 180]
     */
    private double wrapDegrees(double angle) {
        angle %= 360;               // reduce to (-360, 360)
        if (angle > 180)  angle -= 360;  // fold [180, 360) to [-180, 0)
        if (angle < -180) angle += 360;  // fold (-360, -180) to (0, 180]
        return angle;
    }

    // -----------------------------------------------------------------------
    // Public control method
    // -----------------------------------------------------------------------

    /**
     * Commands the servo to move toward the given target angle, using a PID
     * controller to compute the required power.
     *
     * <p>Call this method every loop iteration while position control is active.
     * The servo will move toward {@code target} and settle when the error falls
     * within the {@link #deadband}.
     *
     * @param target desired shaft angle in degrees (wrapped internally to [-180, 180])
     */
    public void moveToAngle(double target) {
        // Read current angle from the (filtered) encoder.
        double current = voltageToAngle(getFilteredVoltage());

        // Compute the shortest-path error from current to target.
        double error = wrapDegrees(target - current);

        // ---- Deadband / hold phase ----
        if (Math.abs(error) < deadband) {
            // Close enough: apply a small hold power in the direction of the last
            // recorded error to resist backdrive forces.
            crServo.setPower(holdPower * Math.signum(lastError));
            lastError = error; // update for next iteration
            return;            // skip PID computation; we're within tolerance
        }

        // ---- Full PID phase ----
        double dt = timer.seconds(); // elapsed time since the last call (seconds)
        timer.reset();               // restart the timer for the next iteration
        if (dt < 0.0001) dt = 0.0001; // guard against near-zero dt (first call or very fast loop)

        // Accumulate error for the integral term and clamp to prevent wind-up.
        integral += error * dt;
        integral = Math.max(-2, Math.min(2, integral)); // clamp integral to [-2, 2]

        // Derivative: rate of change of error since the previous call.
        double derivative = (error - lastError) / dt;

        // Combine PID terms.
        double output = kp * error + ki * integral + kd * derivative;

        // Enforce a minimum power to overcome static friction (stiction).
        if (Math.abs(output) < minPower)
            output = minPower * Math.signum(output); // preserve direction but boost magnitude

        // Clamp final output to valid servo power range [-1, 1].
        output = Math.max(-1.0, Math.min(1.0, output));

        crServo.setPower(output); // apply computed power to the servo

        lastError = error; // save current error for next derivative calculation
    }
}
