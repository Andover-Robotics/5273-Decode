package org.firstinspires.ftc.teamcode.auto.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.AngularVelConstraint;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.MinVelConstraint;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

import java.util.Arrays;

@Config
        @Autonomous(name = "Testing opmode", group = "Autonomous")
        public class TestingOpmode extends LinearOpMode {

            public static int row = 1;
            public static int id = 21;

            public static double maxVel1 = 10;

            @Override
            public void runOpMode() {
                Hardware hardware = new Hardware(hardwareMap, telemetry, this);
                BotActions botActions = hardware.actions;
                MecanumDrive drive = hardware.mecanumDrive;


                Pose2d poseStart = new Pose2d(0, 0, Math.toRadians(0));
                Pose2d pose3 = new Pose2d(0, 72, Math.toRadians(0));

                VelConstraint velConstraint1 = new MinVelConstraint(Arrays.asList(new TranslationalVelConstraint(maxVel1)));
                TranslationalVelConstraint velConstraint2 = new TranslationalVelConstraint(maxVel1);

                Action testSomething = new SequentialAction(
                        new ParallelAction(

                                drive.actionBuilder(poseStart)
                                        .strafeToLinearHeading(pose3.position, pose3.heading, velConstraint2)
                                        .build()/*,

                                new SequentialAction(
                                        new SleepAction(1),
                                        botActions.initializeAuto(Indexer.IndexerState.two),
                                        new SleepAction(1),
                                        botActions.rotateToMotifColorBeforeOuttake(row, id, 0),
                                        new SleepAction(3)
                                )*/
                        )
                );

                waitForStart();
                if (isStopRequested()) return;

                Actions.runBlocking(
                        new ParallelAction(
                                botActions.actionPeriodic(),
                                new SequentialAction(
                                        testSomething
                                )
                        )
                );
            }
}