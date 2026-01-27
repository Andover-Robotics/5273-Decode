package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "OneDriver", group = "AA_main")
public class OneDriverTeleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        Bot bot = new Bot(hardwareMap, telemetry, gamepad1, gamepad2,false);
        bot.teleopInit();
        waitForStart();
        bot.teleopStart();
        while (opModeIsActive() && !isStopRequested()) {
            bot.teleopTick();
        }
    }
}