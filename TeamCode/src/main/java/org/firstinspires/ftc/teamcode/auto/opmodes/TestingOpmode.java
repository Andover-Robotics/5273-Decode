package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

        @Config
        @Autonomous(name = "Testing opmode", group = "Autonomous")
        public class TestingOpmode extends LinearOpMode {

            public static double OBELISK_X = 5;
            public static double OBELISK_Y = 24;
            public static double OBELISK_HEADING_DEG = -60;

            public static double SHOOT_X = 12;
            public static double SHOOT_Y = 42;
            public static double SHOOT_HEADING_DEG = -135;

            public static double INTAKE_X = 5;
            public static double INTAKE1_Y = 51;
            public static double INTAKE2_Y = 75;
            public static double INTAKE_FORWARD_DIST = 8;

            public static double PARK_X = 6;
            public static double PARK_Y = 68;

            public static int SHOOT_RPM = 4010;
            public static int timeToStartOuttakeBeforeToOuttake = 1;

            // has quick outtake and quick intake
            @Override
            public void runOpMode() {
                Hardware hardware = new Hardware(hardwareMap, telemetry);
                BotActions botActions = new BotActions(
                        telemetry,
                        hardware.intake,
                        hardware.indexer,
                        hardware.outtake,
                        hardware.actuator,
                        hardware.aprilTag,
                        hardware.aprilAimer
                );

                Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
                MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

                Pose2d obeliskPose = new Pose2d(
                        OBELISK_X,
                        OBELISK_Y,
                        Math.toRadians(OBELISK_HEADING_DEG)
                );

                Pose2d shootingPose = new Pose2d(
                        SHOOT_X,
                        SHOOT_Y,
                        Math.toRadians(SHOOT_HEADING_DEG)
                );

                Pose2d intake1PoseStart = new Pose2d(INTAKE_X, INTAKE1_Y, Math.toRadians(180));
                Pose2d intake1Pose3 = new Pose2d(INTAKE_X - 3 * INTAKE_FORWARD_DIST - 4, INTAKE1_Y, Math.toRadians(180));

                Pose2d intake2PoseStart = new Pose2d(INTAKE_X, INTAKE2_Y, Math.toRadians(180));
                Pose2d intake2Pose3 = new Pose2d(INTAKE_X - 3 * INTAKE_FORWARD_DIST - 4.5, INTAKE2_Y, Math.toRadians(180));
                Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(180));

                Action testSomething = new SequentialAction(
                        new ParallelAction(
                                drive.actionBuilder(startPose)
                                        .strafeToLinearHeading(startPose.position, startPose.heading)
                                        .build(),

                                new SequentialAction(
                                        new InstantAction(botActions::initializeForIntake),
                                        new SleepAction(1),
                                        botActions.actionQuickOuttake(3000)
                                )
                        )
                );

        waitForStart();
        if (isStopRequested()) return;

        Thread periodicThread = new Thread(() -> {
            while (!Thread.currentThread().isInterrupted() && opModeIsActive()) {
                botActions.actionPeriodic().run(null);
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        });
        periodicThread.start();

        Actions.runBlocking(
                new SequentialAction(
                        testSomething
                )
        );

        periodicThread.interrupt();
        try {
            periodicThread.join();
        } catch (InterruptedException ignored) {
        }
    }
}