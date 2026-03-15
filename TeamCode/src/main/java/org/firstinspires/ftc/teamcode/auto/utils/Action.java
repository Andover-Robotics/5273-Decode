package org.firstinspires.ftc.teamcode.auto.utils;

import java.util.ArrayList;
import java.util.List;

/**
 * Action is a lightweight sequential-task runner for use in autonomous
 * routines that do not use the full Road Runner Action framework.
 *
 * <p>An {@code Action} holds an ordered list of {@link Runnable} steps.
 * Calling {@link #run()} executes every step in sequence, blocking until
 * all steps complete.
 *
 * <p>Instances are constructed exclusively through the nested {@link Builder}
 * to enforce a clear, readable construction pattern:
 * <pre>
 *   Action myAction = new Action.Builder()
 *       .add(() -> subsystem.doSomething())
 *       .moveTo(() -> drive.moveTo(targetPose))
 *       .add(() -> shooter.fire())
 *       .build();
 *   myAction.run();
 * </pre>
 *
 * <p><b>Note:</b> This class predates the Road Runner-based
 * {@link com.acmerobotics.roadrunner.Action} pipeline and is used in
 * helper utilities that need a simple imperative list of operations.
 */
public class Action {

    // -----------------------------------------------------------------------
    // Steps
    // -----------------------------------------------------------------------

    /**
     * The ordered list of {@link Runnable} steps to execute when {@link #run()}
     * is called.  Populated only by the {@link Builder}.
     */
    private final List<Runnable> steps;

    // -----------------------------------------------------------------------
    // Constructor (private – use Builder)
    // -----------------------------------------------------------------------

    /**
     * Private constructor; only the {@link Builder} may create Action instances.
     *
     * @param steps the ordered list of steps to execute
     */
    // Constructor is private; only Builder can build Actions
    private Action(List<Runnable> steps) {
        this.steps = steps; // store the immutable-from-outside step list
    }

    // -----------------------------------------------------------------------
    // Execution
    // -----------------------------------------------------------------------

    /**
     * Executes all steps in order, blocking until each step completes.
     *
     * <p>Steps are run synchronously on the calling thread; there is no
     * timeout or interruption mechanism.
     */
    // Run all steps in order
    public void run() {
        for (Runnable step : steps) {
            step.run(); // execute each step sequentially
        }
    }

    // -----------------------------------------------------------------------
    // Builder
    // -----------------------------------------------------------------------

    /**
     * Fluent builder for constructing {@link Action} instances.
     *
     * <p>Steps can be added in any order using {@link #add(Runnable)} or
     * {@link #moveTo(Runnable)}.  Call {@link #build()} to produce the
     * immutable {@link Action}.
     */
    // Builder nested class
    public static class Builder {

        /** Accumulates steps in insertion order. */
        private final List<Runnable> steps = new ArrayList<>();

        /**
         * Creates an empty Builder.
         */
        public Builder() {}

        /**
         * Appends a custom step to the action sequence.
         *
         * @param step any {@link Runnable} to execute at this position in the sequence
         * @return this Builder (enables method chaining)
         */
        // Add a custom step
        public Builder add(Runnable step) {
            steps.add(step); // append to the ordered step list
            return this;     // return this for chaining
        }

        /**
         * Appends a movement step to the action sequence.
         * Semantically equivalent to {@link #add(Runnable)}; provided as a
         * named convenience to make movement steps visually distinct in code.
         *
         * @param movement the movement command to execute (e.g., a drive-to-pose call)
         * @return this Builder (enables method chaining)
         */
        // Example: move to a position
        public Builder moveTo(Runnable movement) {
            steps.add(movement); // append the movement command like any other step
            return this;
        }

        /**
         * Finalises the builder and returns the constructed {@link Action}.
         * The returned Action's step list is a snapshot of all steps added so far.
         *
         * @return a new {@link Action} containing all added steps in order
         */
        // Build the org.firstinspires.ftc.teamcode.auto.utils.Action
        public Action build() {
            return new Action(steps); // construct the Action from the accumulated steps
        }
    }
}

