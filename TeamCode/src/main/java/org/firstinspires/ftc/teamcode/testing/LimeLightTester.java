package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.LimeLight;

@TeleOp(name="limelight tester")
public class LimeLightTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        LimeLight limelight = new LimeLight(hardwareMap);
        limelight.start();
        waitForStart();
        while (opModeIsActive()) {
            limelight.update();
            telemetry.addData("detected", limelight.detected());
            telemetry.addData("size", limelight.getSize());
            telemetry.addData("x offset", limelight.getXOffset());
            telemetry.addData("y offset", limelight.getYOffset());
            telemetry.update();
        }
    }
}
