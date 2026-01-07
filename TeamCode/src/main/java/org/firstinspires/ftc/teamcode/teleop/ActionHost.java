package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.roadrunner.Action;

public class ActionHost {
    protected Action current;

    public void start(Action action) {
        current = action;
    }

    public void abort() {
        current = null;
    }

    public boolean isRunning() {
        return current != null;
    }

    public void update() {
        if (current == null) return;

        boolean stillRunning = current.run(null);
        if (!stillRunning) {
            current = null;
        }
    }
}