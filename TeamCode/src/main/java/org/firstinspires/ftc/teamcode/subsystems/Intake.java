package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Intake manages the robot's ground intake roller motor.
 *
 * <p>The intake roller sweeps game elements (artifacts) from the field floor
 * into the indexer drum.  Positive "intaking" direction is configured so that
 * setting power to {@link #INTAKING_POWER} (a negative value) spins the motor
 * in the direction that pulls artifacts in.  Reversing the sign ejects artifacts
 * back onto the field, which is useful for clearing jams.
 *
 * <p>A {@link #SLOW_MULTIPLIER} is provided so that, when the indexer is
 * actively aligning a freshly-collected artifact, the intake can be slowed
 * down to prevent forcing a second artifact in before the first has seated.
 *
 * <p>Annotated with {@link Config} so {@code SLOW_MULTIPLIER} and
 * {@code INTAKING_POWER} can be tuned live via FTC Dashboard.
 */
@Config
public class Intake {

    // -----------------------------------------------------------------------
    // Dashboard-tunable power constants
    // -----------------------------------------------------------------------

    /** Fraction (0–1) applied to {@link #INTAKING_POWER} when running at reduced
     *  speed.  A value of 0.5 means 50 % of full intake power.  Used to slow the
     *  roller while the indexer repositions so a second artifact doesn't overrun
     *  a freshly-collected one. */
    public static double SLOW_MULTIPLIER = 0.5;

    /** Raw motor power applied during normal full-speed intaking.  Negative because
     *  the motor's forward direction physically ejects artifacts; reversing it pulls
     *  them into the robot. */
    public static double INTAKING_POWER = -1;

    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------

    /** FTCLib extended motor wrapper for the intake roller, mapped to the
     *  hardware config name {@code "intake"}. */
    private MotorEx intakeMotor;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Creates an Intake instance and looks up the motor in the hardware map.
     *
     * @param hardwareMap the robot's hardware map used to resolve the motor
     *                    registered under the configuration name {@code "intake"}
     */
    public Intake(HardwareMap hardwareMap)
    {
        // Retrieve the intake roller motor from the hardware map using its
        // config name.  MotorEx wraps DcMotorEx with additional helper methods.
        intakeMotor = new MotorEx(hardwareMap, "intake");
    }

    // -----------------------------------------------------------------------
    // Control methods
    // -----------------------------------------------------------------------

    /**
     * Stops the intake motor immediately by cutting power to zero.
     * Should be called whenever the intake is no longer needed to prevent
     * unnecessary wear and to avoid accidentally ingesting extra artifacts.
     */
    public void stop()
    {
        intakeMotor.stopMotor(); // set motor power to 0 and coast to a stop
    }

    /**
     * Runs the intake at full speed in the intaking direction ({@link #INTAKING_POWER}).
     * Use this during active field collection when the robot drives over artifacts.
     */
    public void run()
    {
        intakeMotor.set(INTAKING_POWER); // full-speed intake (pulls artifacts inward)
    }

    /**
     * Runs the intake at reduced speed ({@link #INTAKING_POWER} ×
     * {@link #SLOW_MULTIPLIER}).  Useful as a "passive hold" mode while the
     * indexer positions an artifact that was just collected, preventing a second
     * artifact from ramming in before the indexer is ready.
     */
    public void runSlow() {
        intakeMotor.set(INTAKING_POWER * SLOW_MULTIPLIER); // half-speed intake
    }

    /**
     * Runs the intake in reverse at full speed, ejecting artifacts back onto
     * the field.  Useful for clearing jams or returning incorrect pieces.
     */
    public void runBackwards()
    {
        // Negating INTAKING_POWER flips the motor direction to "eject"
        intakeMotor.set(-INTAKING_POWER);
    }

    /**
     * Runs the intake in reverse at reduced speed ({@code -INTAKING_POWER ×
     * SLOW_MULTIPLIER}).  Provides a gentler eject for careful jam-clearing
     * without flinging artifacts too far.
     */
    public void runBackwardsSlow() {
        intakeMotor.set(-INTAKING_POWER * SLOW_MULTIPLIER); // slow reverse eject
    }

    /**
     * Sets the intake motor to an arbitrary power level.
     * Useful for fine-tuning or for custom autonomous sequences that need a
     * specific power value not covered by the preset methods above.
     *
     * @param newPower motor power in the range [-1.0, 1.0]; positive pushes
     *                 artifacts out, negative pulls them in (given the motor
     *                 convention set in {@link #INTAKING_POWER})
     */
    public void setPower(double newPower)
    {
        intakeMotor.set(newPower); // directly write the requested power to the motor
    }

    /**
     * Returns the current power being applied to the intake motor.
     * Used in telemetry to verify that the intake is running at the expected level.
     *
     * @return current motor power in the range [-1.0, 1.0]
     */
    public double getPower()
    {
        return intakeMotor.get(); // read back the last commanded power from the motor wrapper
    }
}
