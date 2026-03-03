package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Loop Time Tester", group = "testing")
class LoopTimeTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        long lastTime;
        waitForStart();
        lastTime = System.currentTimeMillis();
        while (opModeIsActive())
        {
            telemetry.addData("Loop Time (ms): ", (System.currentTimeMillis() - lastTime));
            lastTime = System.currentTimeMillis();
        }
    }
}