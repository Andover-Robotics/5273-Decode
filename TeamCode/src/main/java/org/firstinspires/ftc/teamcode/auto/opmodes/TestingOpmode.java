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

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;

import java.util.function.IntSupplier;

@Config
@Autonomous(name = "Testing opmode", group = "Autonomous")
public class TestingOpmode extends LinearOpMode {
    private Hardware hardware;

    public static int row = 1;
    public static IntSupplier id = new IntSupplier() {
        @Override
        public int getAsInt() {
            return 21;
        }
    };

    public static double maxVel1 = 10;

    public static boolean isFarShooting = false;

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Hardware hardware = new Hardware(hardwareMap, telemetry, startPose);
        BotActions botActions = new BotActions(hardware, telemetry, this);

        MecanumDrive drive = hardware.mecanumDrive;

        //TranslationalVelConstraint velConstraint1 = new TranslationalVelConstraint(maxVel1);

        Pose2d poseStart = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d endPose = new Pose2d(1, 1, Math.toRadians(1));


        Action intakeThreeAction;


        Action testSomething = new SequentialAction(
                new SequentialAction(
                        //botActions.actionSetSomeShizzle(),
                        new SleepAction(2),
                        /*botActions.actionSetIntakeReverse(),
                        new SleepAction(2)*/
                        //botActions.rotateToMotifColorBeforeOuttake(row, id, 2)
                        botActions.actionOuttake()
                        /*drive.actionBuilder(poseStart)
                                .strafeToLinearHeading(endPose.position, endPose.heading, velConstraint1)
                                .build()*/

                )
        );

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        /*new SequentialAction(
                                new SleepAction(0.5),
                                botActions.actionScanObelisk()
                        ),*/
                        botActions.actionPeriodic(isFarShooting),
                        new SequentialAction(
                                testSomething/*,
                                intakeThreeAction*/
                        )
                )
        );
    }
}
