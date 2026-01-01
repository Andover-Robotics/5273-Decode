package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Indexer.IndexerState;

@Config
@TeleOp(name = "Color Finder", group = "Testing")
public class ColorTester extends OpMode {

    private Indexer indexer;
    GamepadEx gp2;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        indexer = new Indexer(hardwareMap);
        indexer.setTelemetry(telemetry);   // <-- THIS

        telemetry.addLine("Indexer Debug Initialized");
        telemetry.addLine("Rotate indexer by hand");
        telemetry.update();

        gp2 = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {

        gp2.readButtons();


        // Run logic
        indexer.update();

        double angle = indexer.getMeasuredAngle();
        IndexerState closest = indexer.debugClosestSlot();

        telemetry.addLine("===== INDEXER STATE =====");
        telemetry.addData("Measured Angle (deg)", "%.2f", angle);
        telemetry.addData("Intaking Mode", indexer.isIntaking());
        telemetry.addLine();
        telemetry.addLine("===== SLOT ALIGNMENT =====");
        telemetry.addData("Closest Slot", closest);

        telemetry.addLine();
        telemetry.addLine("===== SLOT CONTENTS =====");
        for (IndexerState s : IndexerState.values()) {
            telemetry.addData(
                    "Slot " + s.index,
                    "%s  (err=%.1f°)",
                    indexer.getColorAt(s),
                    indexer.debugSlotErrorDeg(s)
            );
        }

        telemetry.addLine();
        telemetry.addLine("===== SERVO DEBUG =====");
        telemetry.addData("Voltage", "%.3f", indexer.getVoltage());
        telemetry.addData("Target Voltage", "%.3f", indexer.getTargetVoltage());

        telemetry.addLine();
        telemetry.addLine("===== INSTRUCTIONS =====");
        telemetry.addLine("• Rotate indexer slowly by hand");
        telemetry.addLine("• Insert balls while slot is over sensor");
        telemetry.addLine("• Leave slot → observe classification");
        telemetry.addLine("• Try intake vs outtake modes");

        if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            indexer.moveTo(indexer.getState().next());
        }

        if(gp2.wasJustPressed(GamepadKeys.Button.A))
            indexer.moveToColor(Indexer.ArtifactColor.PURPLE);
        if(gp2.wasJustPressed(GamepadKeys.Button.B))
            indexer.moveToColor(Indexer.ArtifactColor.GREEN);

        telemetry.update();
    }
}
