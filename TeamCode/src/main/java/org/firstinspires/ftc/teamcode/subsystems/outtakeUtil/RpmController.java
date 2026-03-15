package org.firstinspires.ftc.teamcode.subsystems.outtakeUtil;

import static androidx.core.math.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;

/**
 * RpmController is a feedforward + proportional (FP) controller used to drive
 * the shooter flywheel motors to a target RPM.
 *
 * <p>The control law is:
 * <pre>
 *   ff    = kS * sign(setpoint) + kV * setpoint   // static + velocity feedforward
 *   pOut  = Kp * (setpoint - measured)            // proportional error correction
 *   output = clamp(ff + pOut, -1.0, 1.0)          // combined, clamped to motor range
 * </pre>
 *
 * <p>The feedforward component ({@code kS} + {@code kV}) pre-loads the motor
 * with approximately the right power so that the proportional term ({@code Kp})
 * only needs to correct small residual errors.  This reduces overshoot relative
 * to pure P-only control while still reaching the setpoint quickly.
 *
 * <p>Annotated with {@link Config} so gains can be tuned live in FTC Dashboard
 * (note: the fields are {@code final} and will not actually be editable at
 * runtime unless changed to {@code public static}).
 */
@Config
public class RpmController {

    // -----------------------------------------------------------------------
    // Gains (set once at construction via the constructor)
    // -----------------------------------------------------------------------

    /**
     * Proportional gain: scales the RPM error (setpoint − measured) into a
     * motor power correction.  Larger values close the error faster but may
     * cause overshoot or oscillation if too high.
     */
    public final double Kp;

    /**
     * Static friction feedforward gain: a small constant power added in the
     * direction of the setpoint to overcome motor and gearbox stiction before
     * the proportional term fully engages.  Ensures the flywheel starts moving
     * even at very low RPM targets.
     */
    public final double kS; // static (stiction) feedforward

    /**
     * Velocity feedforward gain: scales the setpoint RPM directly into a base
     * motor power estimate.  The product {@code kV * setpointRPM} approximates
     * the power needed to sustain the target speed, leaving only small errors
     * for the P term to handle.
     */
    public final double kV; // velocity proportional feedforward

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Creates an RpmController with the given FP gains.
     *
     * @param Kp proportional gain (RPM error → motor power)
     * @param kS static-friction feedforward gain
     * @param kV velocity feedforward gain (RPM → motor power)
     */
    public RpmController(double Kp, double kS, double kV) {
        this.Kp = Kp; // store proportional gain
        this.kS = kS; // store static feedforward gain
        this.kV = kV; // store velocity feedforward gain
    }

    // -----------------------------------------------------------------------
    // Control update
    // -----------------------------------------------------------------------

    /**
     * Computes the motor power needed to drive the flywheel from
     * {@code measuredRPM} toward {@code setpointRPM} using feedforward + P
     * control.
     *
     * <p>Algorithm:
     * <ol>
     *   <li>Compute RPM error = setpoint − measured.</li>
     *   <li>Compute feedforward = kS × sign(setpoint) + kV × setpoint.</li>
     *   <li>Compute P correction = Kp × error.</li>
     *   <li>Sum and clamp to [−1, 1].</li>
     * </ol>
     *
     * @param setpointRPM desired shooter flywheel speed in RPM
     * @param measuredRPM current measured flywheel speed in RPM (from encoder)
     * @return commanded motor power in the range [−1.0, 1.0]
     */
    public double update(double setpointRPM, double measuredRPM) {
        // RPM error: positive means we need to speed up, negative to slow down.
        double error = setpointRPM - measuredRPM;

        // Feedforward: static term ensures the motor starts moving (overcomes stiction),
        // velocity term pre-loads the output with the approximate sustaining power.
        double ff = kS * Math.signum(setpointRPM) + kV * setpointRPM;

        // Proportional correction on top of feedforward to close residual RPM error.
        double pOut = Kp * error;

        // Sum feedforward and P correction, then clamp to the valid motor power range.
        double output = clamp(ff + pOut, -1.0, 1.0);
        return output; // return the combined motor power command
    }
}
