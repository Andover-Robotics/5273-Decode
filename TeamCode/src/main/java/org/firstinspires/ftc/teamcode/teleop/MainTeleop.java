package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

@TeleOp(name = "AA_MainTeleOp", group = "AA_main")
public class MainTeleop extends LinearOpMode {

    public static Pose2d startPose = new Pose2d(0, 0, 0);

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap,startPose);
        Bot bot = new Bot(hardwareMap, telemetry, drive, gamepad1, gamepad2,false);
        bot.teleopInit();
        waitForStart();
        bot.teleopStart();
        while (opModeIsActive() && !isStopRequested()) {
            bot.teleopTick();
        }
    }
}