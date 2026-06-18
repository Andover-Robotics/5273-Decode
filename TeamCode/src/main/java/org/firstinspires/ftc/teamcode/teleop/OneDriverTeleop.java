package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

@TeleOp(name = "OneDriver", group = "AA_main")
public class OneDriverTeleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0)));
        Bot bot = new Bot(hardwareMap, telemetry, drive, gamepad1, gamepad2,true, false);
        bot.teleopInit();
        waitForStart();
        bot.teleopStart();

        boolean isFarShooting = false;
        while (opModeIsActive() && !isStopRequested()) {
            bot.teleopTick(isFarShooting);
        }
    }
}