package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

/**
 * ActionHost manages the execution of a single Road Runner {@link Action} at
 * a time in the teleop loop.
 *
 * <p>Road Runner {@link Action}s are designed for use in autonomous sequences,
 * but they can also be used during teleop to perform multi-step background
 * operations (e.g., raise the actuator, wait for RPM, fire, lower actuator)
 * without blocking the main teleop loop.
 *
 * <p>Usage:
 * <ol>
 *   <li>Call {@link #start(Action)} to begin executing an action.</li>
 *   <li>Call {@link #update()} every loop iteration.  Each call advances the
 *       action by one step and sends telemetry to the FTC Dashboard.</li>
 *   <li>The action ends automatically when it returns {@code false} from
 *       {@link Action#run(TelemetryPacket)}, or it can be cancelled early
 *       with {@link #abort()}.</li>
 *   <li>Check {@link #isRunning()} to decide whether to start a new action or
 *       allow driver inputs that would conflict with the current one.</li>
 * </ol>
 *
 * <p>Only one action runs at a time.  Starting a new action via
 * {@link #start(Action)} while one is running will replace the current action.
 */
public class ActionHost {

    // -----------------------------------------------------------------------
    // State
    // -----------------------------------------------------------------------

    /**
     * The currently executing {@link Action}, or {@code null} if no action is
     * active.  Made {@code protected} so subclasses can inspect it if needed.
     */
    protected Action current;

    // -----------------------------------------------------------------------
    // Control methods
    // -----------------------------------------------------------------------

    /**
     * Starts executing the given {@link Action}.
     * If an action is already running it is discarded and replaced by the new
     * one immediately.
     *
     * @param action the Road Runner Action to execute; must not be {@code null}
     */
    public void start(Action action) {
        current = action; // replace any in-progress action with the new one
    }

    /**
     * Cancels the currently running action.
     * After this call {@link #isRunning()} returns {@code false} and
     * {@link #update()} becomes a no-op until {@link #start(Action)} is called
     * again.
     */
    public void abort() {
        current = null; // discard the in-progress action; any hardware it was driving keeps its last power
    }

    /**
     * Returns whether an action is currently executing.
     * Use this in the teleop loop to guard button presses that would conflict
     * with a running action sequence.
     *
     * @return {@code true} if an action is in progress; {@code false} otherwise
     */
    public boolean isRunning() {
        return current != null; // non-null current ↔ action running
    }

    // -----------------------------------------------------------------------
    // Loop update
    // -----------------------------------------------------------------------

    /**
     * Advances the current action by one loop iteration.
     * Must be called every robot loop tick (typically inside
     * {@code BotPeriodics.handlePeriodics()}).
     *
     * <p>Each call:
     * <ol>
     *   <li>If no action is running, returns immediately.</li>
     *   <li>Creates a new {@link TelemetryPacket} for the Dashboard.</li>
     *   <li>Calls {@link Action#run(TelemetryPacket)} on the current action.
     *       The action executes one step and returns {@code true} if it should
     *       continue, {@code false} if it has completed.</li>
     *   <li>Sends the telemetry packet to FTC Dashboard so live data is visible.</li>
     *   <li>If the action is complete ({@code run()} returned {@code false}),
     *       clears {@link #current} to mark the host as idle.</li>
     * </ol>
     */
    public void update() {
        if (current == null) return; // no action running; nothing to do this iteration

        // Create a fresh telemetry packet; the action can write debug data into it.
        TelemetryPacket packet = new TelemetryPacket();

        // Execute one step of the action.  Returns true while still running, false when done.
        boolean stillRunning = current.run(packet);

        // Forward the packet to FTC Dashboard so live telemetry is visible during the action.
        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        if (!stillRunning) {
            current = null; // action finished; clear the host so new actions can be started
        }
    }
}