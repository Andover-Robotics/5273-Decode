package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Turret models a rotational (panning) platform driven by a single DC motor.
 *
 * <p>The turret can be used to aim a sensor or shooter by rotating it to a
 * desired angle.  Angle calculation uses a ticks-per-degree conversion factor
 * ({@link #TICKS_TO_DEGREES}) and a zero-position offset ({@link #ZERO_OFFSET})
 * that compensate for the motor's encoder starting point and any mechanical
 * offset between "encoder zero" and the physical zero of the turret.
 *
 * <p>The current angle is always wrapped into the [0, 360) range to avoid
 * ambiguity about "which revolution" the turret is on.
 *
 * <p>Annotated with {@link Config} so {@link #TICKS_TO_DEGREES} and
 * {@link #ZERO_OFFSET} can be calibrated live in FTC Dashboard.
 */
@Config
public class Turret {

    // -----------------------------------------------------------------------
    // Dashboard-tunable calibration constants
    // -----------------------------------------------------------------------

    /** Conversion factor from raw encoder ticks to degrees of turret rotation.
     *  Set to 0 by default; must be measured and configured for the specific
     *  gear ratio and encoder used on this robot. */
    public static double TICKS_TO_DEGREES = 0;

    /** Angular offset (degrees) added to the converted tick value to align
     *  the encoder's zero position with the desired physical zero of the turret.
     *  A positive offset rotates the "zero" clockwise. */
    public static double ZERO_OFFSET = 0;

    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------

    /** The DC motor that drives the turret rotation mechanism.
     *  Mapped to hardware config name {@code "TurretMotor"}. */
    private final DcMotor motor;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Creates a Turret instance, retrieves the motor from the hardware map,
     * and configures it to BRAKE when power is zero (so the turret holds its
     * position without drifting when no correction is applied).
     *
     * @param hardwareMap the robot's hardware map used to look up the motor
     *                    registered under the config name {@code "TurretMotor"}
     */
    public Turret(HardwareMap hardwareMap) {
        // Retrieve the turret motor from the hardware map.
        motor = hardwareMap.get(DcMotor.class, "TurretMotor");

        // BRAKE mode holds the turret in place when power = 0, preventing wind-up.
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor direction reversal may be needed depending on how the motor is mounted;
        // uncomment the line below and adjust if the turret spins the wrong way.
        // motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /**
     * Returns the current turret angle in degrees, wrapped to [0, 360).
     *
     * <p>The raw encoder tick count is first multiplied by {@link #TICKS_TO_DEGREES}
     * to convert to degrees, then {@link #ZERO_OFFSET} is added to correct for
     * any mechanical misalignment.  Finally the result is wrapped with a
     * positive-modulo operation so that the angle is always in [0, 360).
     *
     * <p><b>Note:</b> Java's {@code %} operator returns a negative result for
     * negative operands (e.g., {@code -2 % 5 == -2}), so the double-modulo
     * pattern {@code ((x % 360) + 360) % 360} is used to guarantee a positive
     * result.
     *
     * @return current turret angle in degrees, range [0, 360)
     */
    public double getCurrentAngle() {
        // Convert encoder ticks to degrees and apply the zero-point offset.
        double curAngle = motor.getCurrentPosition() * TICKS_TO_DEGREES + ZERO_OFFSET;
        // In Java -2 % 5 = -2, not 3
        // Wrap into [0, 360): add 360 first then take mod again to handle negative values.
        return ((curAngle % 360) + 360) % 360;
    }

    /**
     * Sets the raw power applied to the turret motor.
     * A positive value rotates in one direction; a negative value reverses it.
     * Callers are responsible for implementing closed-loop angle control on
     * top of this method.
     *
     * @param power motor power in the range [-1.0, 1.0]
     */
    public void setPower(double power) {
        motor.setPower(power); // directly apply power to the turret motor
    }
}