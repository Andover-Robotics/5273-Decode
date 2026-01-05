package org.firstinspires.ftc.teamcode.auto.helpers;

import java.util.ArrayList;
import java.util.List;

public class Action {

    private final List<Runnable> steps;

    // Constructor is private; only Builder can build Actions
    private Action(List<Runnable> steps) {
        this.steps = steps;
    }

    // Run all steps in order
    public void run() {
        for (Runnable step : steps) {
            step.run();
        }
    }

    // Builder nested class
    public static class Builder {
        private final List<Runnable> steps = new ArrayList<>();

        public Builder() {}

        // Add a custom step
        public Builder add(Runnable step) {
            steps.add(step);
            return this;  // Allows chaining
        }

        // Example: move to a position
        public Builder moveTo(Runnable movement) {
            steps.add(movement);
            return this;
        }

        // Build the org.firstinspires.ftc.teamcode.auto.helpers.Action
        public Action build() {
            return new Action(steps);
        }
    }
}

