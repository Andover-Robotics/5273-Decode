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
import java.util.function.IntSupplier;

@Config
        @Autonomous(name = "Testing opmode", group = "Autonomous")
        public class TestingOpmode extends LinearOpMode {

            public static int row = 1;
            public static IntSupplier id = new IntSupplier() {
                @Override
                public int getAsInt() {
                    return 21;
                }
            };

            public static double maxVel1 = 10;

            @Override
            public void runOpMode() {
                Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

                Hardware hardware = new Hardware(hardwareMap, telemetry, this, startPose);
                BotActions botActions = hardware.actions;

                MecanumDrive drive = hardware.mecanumDrive;

                //TranslationalVelConstraint velConstraint1 = new TranslationalVelConstraint(maxVel1);

                Pose2d poseStart = new Pose2d(0, 0, Math.toRadians(0));
                Pose2d endPose = new Pose2d(1, 1, Math.toRadians(1));


                Action intakeThreeAction = botActions.actionIntakeThreeFeedback(
                        poseStart,
                        endPose,
                        endPose,
                        drive,
                        maxVel1
                );


                Action testSomething = new SequentialAction(
                        new SequentialAction(
                                botActions.actionSetSomeShizzle(),
                                new SleepAction(2),
                                botActions.actionQuickOuttake(),
                                new SleepAction(2),
                                botActions.rotateToMotifColorBeforeOuttake(row, id, 2)
                                /*drive.actionBuilder(poseStart)
                                        .strafeToLinearHeading(endPose.position, endPose.heading, velConstraint1)
                                        .build()*/

                        )
                );

                waitForStart();
                if (isStopRequested()) return;

                Actions.runBlocking(
                        new ParallelAction(
                                botActions.actionPeriodic(),
                                new SequentialAction(
                                        testSomething/*,
                                        intakeThreeAction*/
                                )
                        )
                );
            }
}
