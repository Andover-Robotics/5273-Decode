package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;

@Config
@Autonomous(name = "Far Leave X Auto", group = "Autonomous")
public class FarLeaveX extends LinearOpMode {
    public static double PARK_X = 24;
    public static double PARK_Y = 0;

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Hardware hardware = new Hardware(hardwareMap, telemetry, startPose);
        BotActions botActions = new BotActions(hardware, telemetry, this);
        MecanumDrive drive = hardware.mecanumDrive;

        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(0));

        Action toPark = drive.actionBuilder(startPose)
                .strafeTo(
                        parkPose.position
                )
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        botActions.actionPeriodic(),
                        new SequentialAction(
                                new InstantAction(() -> drive.localizer.setPose(startPose)),
                                toPark
                        )
                )
        );
    }
}
