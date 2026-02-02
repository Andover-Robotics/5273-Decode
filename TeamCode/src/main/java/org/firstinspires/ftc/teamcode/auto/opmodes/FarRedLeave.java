package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@Config
@Autonomous(name = "Blue Leave Far Auto", group = "Autonomous")
public class FarRedLeave extends LinearOpMode {
    public static double PARK_X = 26;
    public static double PARK_Y = 0;

    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap, telemetry, this);
        BotActions botActions = hardware.actions;
        MecanumDrive drive = hardware.mecanumDrive;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(0));

        Action toPark = drive.actionBuilder(startPose)
                .strafeToSplineHeading(
                        parkPose.position,
                        parkPose.heading
                )
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        botActions.actionPeriodic(),
                        new SequentialAction(
                                toPark
                        )
                )
        );
    }
}
