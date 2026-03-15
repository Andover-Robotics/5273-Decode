package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Actuator controls the physical ramp/deflector servo that lifts game elements
 * (artifacts) from the indexer into the shooter barrel.
 *
 * <p>Three positions are defined:
 * <ul>
 *   <li>{@link #DOWN} – ramp is fully lowered (flush with the platform floor);
 *       used during intake and transit so artifacts can roll into the indexer.</li>
 *   <li>{@link #UP_QUICK} – ramp is raised to a mid-level angle; used during the
 *       "quick/non-indexed" outtake mode where all three artifacts are blasted out
 *       at once without precise indexer positioning.</li>
 *   <li>{@link #UP_INDEXED} – ramp is raised to the highest angle; used during the
 *       "indexed" outtake mode where a single, colour-selected artifact is fired at a
 *       time with the indexer precisely aligned.</li>
 * </ul>
 *
 * <p>The servo is mapped by the hardware configuration name {@code "actuator"} and
 * operates over a 0–360-degree range (FTCLib {@link SimpleServo}).  The position
 * values below are fractional (0.0–1.0) within that range.
 *
 * <p>Annotated with {@link Config} so every {@code public static} field can be
 * tuned live in the FTC Dashboard without redeploying.
 */
@Config
public class Actuator {

    // -----------------------------------------------------------------------
    // Dashboard-tunable servo position constants
    // -----------------------------------------------------------------------

    /** Servo position (fractional 0–1) for the fully-lowered ramp.
     *  At this position the ramp is flush with the floor of the launch platform,
     *  allowing artifacts to sit flat while the robot intakes or drives. */
    public static double DOWN = 0.45;

    /** Servo position (fractional 0–1) for the mid-height "quick outtake" ramp.
     *  Slightly lower than {@link #UP_INDEXED}, used when all artifacts are
     *  dumped at once via full-power indexer blast rather than precise indexing. */
    public static double UP_QUICK = 0.22;

    /** Servo position (fractional 0–1) for the highest "indexed outtake" ramp.
     *  Raises the ramp to the steepest angle so a single aligned artifact slides
     *  directly into the shooter's feed zone during precision firing. */
    public static double UP_INDEXED = 0.13;

    // -----------------------------------------------------------------------
    // State machine enum
    // -----------------------------------------------------------------------

    /**
     * Represents the three discrete positions of the actuator servo.
     * The state is tracked internally so that callers can query the current
     * position without needing to read back hardware values.
     */
    public enum ActuatorState {
        /** Ramp fully lowered – safe for intake and driving. */
        DOWN,
        /** Ramp at mid-height – used for quick/non-indexed shooting. */
        UP_QUICK,
        /** Ramp at maximum height – used for single-artifact indexed shooting. */
        UP_INDEXED
    }

    // -----------------------------------------------------------------------
    // Fields
    // -----------------------------------------------------------------------

    /** Tracks the last commanded position of the actuator servo. */
    private ActuatorState state;

    /** The physical servo hardware object.  Mapped to the name {@code "actuator"}
     *  with a 0–360-degree range via FTCLib's {@link SimpleServo}. */
    private final SimpleServo servo;

    /** Approximate time in seconds for the servo to travel between positions.
     *  Callers that need to wait for the ramp to reach its target before
     *  firing can sleep for this duration. */
    private double waitTime = 0.5; // seconds

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Initialises the Actuator by fetching the servo from the hardware map.
     *
     * @param hardwareMap the robot's hardware map, used to look up the servo
     *                    device registered under the name {@code "actuator"}
     */
    public Actuator(HardwareMap hardwareMap) {
        // Create a SimpleServo wrapper that expects a 0–360° operating range.
        // FTCLib converts the 0–1 fractional position values to the actual
        // PWM range required by the underlying servo hardware.
        servo = new SimpleServo(hardwareMap, "actuator", 0, 360);
    }

    // -----------------------------------------------------------------------
    // Actuator movement methods
    // -----------------------------------------------------------------------

    /**
     * Lowers the ramp to its fully-down position ({@link #DOWN}).
     * Should be called after every shooting sequence or during intake so
     * artifacts can collect properly.
     */
    public void down() {
        servo.setPosition(DOWN); // command servo to the lowered position
        state = ActuatorState.DOWN; // update cached state to reflect new position
    }

    /**
     * Raises the ramp to the default "up" position, which is the indexed
     * (highest) angle.  Delegates to {@link #upIndexed()}.
     *
     * <p>Using indexed as the default ensures that callers who don't care about
     * the specific mode always get the most precise shooting angle.
     */
    public void up() {
        // Default up = indexed (most precise / highest angle)
        upIndexed();
    }

    /**
     * Raises the ramp to the indexed outtake position ({@link #UP_INDEXED}).
     * This is the steepest angle, used when the indexer is precisely aligned
     * so that exactly one artifact enters the shooter barrel.
     */
    public void upIndexed() {
        servo.setPosition(UP_INDEXED); // command servo to the indexed (highest) position
        state = ActuatorState.UP_INDEXED; // update cached state
    }

    /**
     * Raises the ramp to the quick outtake position ({@link #UP_QUICK}).
     * This is a lower angle than {@link #upIndexed()}, used during the
     * "quick dump" sequence where the indexer spins at full power to eject
     * all three artifacts simultaneously.
     */
    public void upQuick() {
        servo.setPosition(UP_QUICK); // command servo to the quick-dump (mid) position
        state = ActuatorState.UP_QUICK; // update cached state
    }

    // -----------------------------------------------------------------------
    // Query methods
    // -----------------------------------------------------------------------

    /**
     * Returns {@code true} if the ramp is in any raised position (i.e. NOT down).
     * Useful for telemetry and guard conditions that check whether the robot is
     * ready to shoot.
     *
     * @return {@code true} if {@link #state} is {@link ActuatorState#UP_QUICK}
     *         or {@link ActuatorState#UP_INDEXED}; {@code false} if it is
     *         {@link ActuatorState#DOWN}
     */
    public boolean isActivated() {
        // The actuator is considered "activated" (raised) whenever it is NOT in the DOWN state.
        return !(state == ActuatorState.DOWN);
    }

    /**
     * Returns the current cached {@link ActuatorState} of the ramp.
     *
     * @return the last position commanded to the actuator servo
     */
    public ActuatorState getState() {
        return state;
    }

    /**
     * Convenience setter that raises or lowers the ramp based on a boolean flag.
     * Calls {@link #up()} when {@code true} (which defaults to indexed mode),
     * and {@link #down()} when {@code false}.
     *
     * @param activate {@code true} to raise the ramp; {@code false} to lower it
     */
    public void set(boolean activate) {
        if (activate) up(); // raise ramp (defaults to indexed/highest position)
        else down();        // lower ramp to intake/transit position
    }

    /**
     * Returns the estimated time in seconds for the servo to complete its
     * travel between positions.  Callers may use this value in a
     * {@link com.acmerobotics.roadrunner.SleepAction} to wait for the ramp
     * to physically reach its target before proceeding.
     *
     * @return servo travel wait time in seconds
     */
    public double getWaitTime() {
        return waitTime; // seconds – used externally to gate action sequences on physical completion
    }
}