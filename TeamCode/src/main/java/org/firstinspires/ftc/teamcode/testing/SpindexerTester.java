package org.firstinspires.ftc.teamcode.testing;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "SpindexerTest", group = "Teleop")
public class SpindexerTester extends LinearOpMode {

    Indexer indexer;
    Intake intake;
    GamepadEx gp2;

    @Override
    public void runOpMode() {
        indexer = new Indexer(hardwareMap);
        intake = new Intake(hardwareMap);
        gp2 = new GamepadEx(gamepad2);

        waitForStart();

        indexer.setIntaking(true);

        while (opModeIsActive()) {
            gp2.readButtons();

            telemetry.addData("CurrentState: ", indexer.getState());
            telemetry.addData("NextState: ", (indexer.getState().next()));

            if (gp2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                indexer.moveTo(indexer.getState().next());
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.A)) {
                indexer.setIntaking(true);
            }

            if (gp2.wasJustPressed(GamepadKeys.Button.B)) {
                indexer.setIntaking(false);
            }

            if (gp2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.01) {
                intake.run();
            } else {
                intake.stop();
            }

            // Call update each loop to continuously control the servo position
            indexer.update();
            telemetry.addData("Indexer Voltage: ", indexer.getVoltage());

            // Update color scanning timing and sensor reading
            //indexer.updateColorScanning();

            telemetry.update();
        }
    }

    @NonNull
    private static String getMainColor(int r, int g, int b) {
        double total = r + g + b;
        double rRatio = r / total;
        double gRatio = g / total;
        double bRatio = b / total;

        if (rRatio > 0.45 && gRatio < 0.35 && bRatio < 0.35) {
            return "Red";
        } else if (rRatio > 0.45 && gRatio > 0.25 && bRatio < 0.20) {
            return "Orange";
        } else if (rRatio > 0.38 && gRatio > 0.38 && bRatio < 0.25) {
            return "Yellow";
        } else if (gRatio > 0.45 && rRatio < 0.35 && bRatio < 0.35) {
            return "Green";
        } else if (bRatio > 0.45 && rRatio < 0.35 && gRatio < 0.35) {
            return "Blue";
        } else if (rRatio > 0.35 && bRatio > 0.35 && gRatio < 0.30) {
            return "Purple";
        } else {
            return "Unclear";
        }
    }
}
