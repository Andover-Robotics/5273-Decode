package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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
@Autonomous(name = "Far Red Far Shooting auto", group = "Autonomous")
public class FarRedFarShooting extends LinearOpMode {
    public static double OBELISK_X = 0;
    public static double OBELISK_Y = 16;

    public static double SHOOT_X = 0;
    public static double SHOOT_Y = 16;
    public static double SHOOT_HEADING_DEG = 75;

    public static double PARK_X = 26;
    public static double PARK_Y = 0;

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(90));

        Hardware hardware = new Hardware(hardwareMap, telemetry, this, startPose);
        BotActions botActions = hardware.actions;
        MecanumDrive drive = hardware.mecanumDrive;

        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(90));
        Pose2d obeliskPose = new Pose2d(OBELISK_X, OBELISK_Y, Math.toRadians(135));

        Action scanObelisk = new ParallelAction(
                drive.actionBuilder(startPose) // obeliskPose
                        .strafeTo(obeliskPose.position)
                        .strafeToSplineHeading(obeliskPose.position, obeliskPose.heading)
                        .build(),

                new SequentialAction(
                        new SleepAction(2),
                        botActions.rotateToMotifColorBeforeOuttake(0, botActions::getObeliskId, 2)
                )
        );

        Action toShoot = new ParallelAction(
                drive.actionBuilder(startPose)
                        .strafeTo(parkPose.position)
                        .build()

                //botActions.actionStartOuttake(SHOOT_RPM),

                );

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
                                botActions.actionScanObelisk()
                        ),
                        new SequentialAction(
                                new InstantAction(() -> drive.localizer.setPose(startPose)),
                                scanObelisk,
                                toShoot,
                                toPark
                        )
                )
        );
    }
}
