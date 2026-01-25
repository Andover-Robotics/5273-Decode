package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.indexerUtil.ColorSensorSystem;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Indexer.IndexerState;

@Config
@TeleOp(name = "Color Finder", group = "Testing")
public class ColorTester extends OpMode {

    private Indexer indexer;
    private ColorSensorSystem colorSensor;
    private GamepadEx gp2;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        indexer = new Indexer(hardwareMap);

        // its so peak
        colorSensor = new ColorSensorSystem(hardwareMap);

        telemetry.addLine("ColorTester Initialized");
        telemetry.update();

        gp2 = new GamepadEx(gamepad2);
    }

    @Override
    public void loop() {
        gp2.readButtons();

        indexer.update();

        double angle = indexer.getMeasuredAngle();
        IndexerState closest = indexer.debugClosestSlot();

        // Read sensor ONCE per loop so values are consistent within this telemetry frame
        float alpha = colorSensor.getAlpha();
        float[] rgb = colorSensor.getRGB();
        float[] hsv = colorSensor.getHSV();

        telemetry.addLine("===== RAW COLOR SENSOR (ALWAYS UPDATING) =====");
        telemetry.addData("Alpha", "%.4f", alpha);
        telemetry.addData("R", "%.4f", rgb[0]);
        telemetry.addData("G", "%.4f", rgb[1]);
        telemetry.addData("B", "%.4f", rgb[2]);
        telemetry.addData("H", "%.1f°", hsv[0]);
        telemetry.addData("S", "%.4f", hsv[1]);
        telemetry.addData("V", "%.4f", hsv[2]);
        telemetry.addData("Has Artifact", colorSensor.hasArtifact());
        telemetry.addData("Classify()", colorSensor.classify());
        telemetry.addData("ClassifyColorOnly()", colorSensor.classifyColorOnly());

        telemetry.addLine();
        telemetry.addLine("===== INDEXER STATE =====");
        telemetry.addData("Measured Angle (deg)", "%.2f", angle);
        telemetry.addData("Intaking Mode", indexer.isIntaking());

        telemetry.addLine();
        telemetry.addLine("===== SLOT ALIGNMENT =====");
        telemetry.addData("Closest Slot", closest);
        telemetry.addData("Closest Slot Err (deg)", "%.2f", indexer.debugClosestSlotErrorDeg());
        telemetry.addData("Over Sensor?", indexer.debugSlotIsOverSensor(closest));

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

        if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            indexer.moveTo(indexer.getState().next());
        }

        if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
            indexer.moveToColor(Indexer.ArtifactColor.PURPLE);
        }
        if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
            indexer.moveToColor(Indexer.ArtifactColor.GREEN);
        }

        telemetry.update();
    }
}