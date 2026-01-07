package org.firstinspires.ftc.teamcode.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
        TelemetryPacket packet = new TelemetryPacket();
        boolean stillRunning = current.run(packet);
        FtcDashboard.getInstance().sendTelemetryPacket(packet);
        if (!stillRunning) {
            current = null;
        }
    }
}