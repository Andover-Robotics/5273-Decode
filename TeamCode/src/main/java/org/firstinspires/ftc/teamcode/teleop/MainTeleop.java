package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

@TeleOp(name = "MainTeleOp", group = "AA_main")
public class MainTeleop extends LinearOpMode {

    public static double startX = 0;
    public static double startY = 0;
    public static double startHeading = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startX, startY, Math.toRadians(startHeading)));
        Bot bot = new Bot(hardwareMap, telemetry, drive, gamepad1, gamepad2,false);
        bot.teleopInit();
        waitForStart();
        bot.teleopStart();
        while (opModeIsActive() && !isStopRequested()) {
            bot.teleopTick();
        }
    }
}