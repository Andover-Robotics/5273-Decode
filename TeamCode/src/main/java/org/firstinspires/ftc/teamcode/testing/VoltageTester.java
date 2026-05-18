package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Voltage Tester")
public class VoltageTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Indexer indexer = new Indexer(hardwareMap);
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("voltage", indexer.getVoltage());
            telemetry.update();
        }
    }
}
