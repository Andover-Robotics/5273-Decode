package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;

/**
 * TeleopConstants is a collection of compile-time (and Dashboard-editable)
 * tuning constants used throughout the teleop OpModes.
 *
 * <p>Grouping constants here (rather than scattering magic numbers through
 * the code) makes it easy to:
 * <ul>
 *   <li>Find and adjust values in one place.</li>
 *   <li>Expose them in FTC Dashboard for live tuning (via {@link Config}).</li>
 * </ul>
 *
 * <p>All fields are {@code public static} (not actually {@code final} at
 * runtime) so that FTC Dashboard can write back updated values.  The
 * {@code final} modifier on the inner class is just syntactic sugar; the
 * fields themselves are mutable through the Dashboard.
 */
@Config
public final class TeleopConstants
{
    // Note: fields use final notation syntactically but are NOT truly final
    // because FTC Dashboard can edit them at runtime through reflection.

    /**
     * Constants that govern gamepad feedback (rumble, LEDs) and input
     * filtering applied to both driver gamepads during teleop.
     */
    public static final class Gamepad {

        /**
         * Duration (milliseconds) for which the gamepad LED colour is held
         * after an alliance-selection button press.  After this time the LED
         * reverts to its default colour.
         */
        public static int GAMEPAD_LIGHT_COLOR_DURATION = 500; // ms

        /**
         * Number of rumble "blips" sent to both gamepads when the indexer
         * drum is completely full.  Three short blips alert both drivers
         * without being overly distracting.
         */
        public static int FULL_WARNING_RUMBLES = 3;

        /**
         * Minimum trigger axis value that counts as a deliberate button press.
         * Values below this threshold are treated as 0.  Prevents unintended
         * intake activation from small amounts of resting-finger pressure on
         * the trigger buttons.
         */
        public static double TRIGGER_DEADZONE = 0.05; // trigger value below this is ignored
    }
    //
}
