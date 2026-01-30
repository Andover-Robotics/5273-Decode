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
@Autonomous(name = "Faster Red Auto No Motif", group = "Autonomous")
public class FasterRedClose extends LinearOpMode {

    public static double SHOOT_X = 15;
    public static double SHOOT_Y = 42;
    public static double SHOOT_HEADING_DEG = -132;
    public static double SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT = 4;

    public static double INTAKE_START_X = 16;
    public static double INTAKE2_START_OFFSET_X = 2.5;
    public static double INTAKE3_START_OFFSET_X = 5;
    public static double INTAKE_END_X = -13;
    public static double intake_END_2And3_XOffset = 6.0;

    public static double INTAKE1_Y = 50;
    public static double INTAKE2_Y = 77;
    public static double INTAKE3_Y = 97;

    public static double gateStart_X = 2;
    public static double gateStart_Y = 77;
    public static double gateEnd_X = -14;
    public static double gateEnd_Y = 63;
    public static double gateWaitTime = 1.5;

    public static double PARK_X = 6;
    public static double PARK_Y = 68;

    public static int SHOOT_RPM = 3380;

    public static double timeUntilStartOuttake = 1.65; // Time until you start the outtake action, which still includes the wait for actuator

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
        MecanumDrive drive = hardware.mecanumDrive;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

        Pose2d shootingPose = new Pose2d(
                SHOOT_X,
                SHOOT_Y,
                Math.toRadians(SHOOT_HEADING_DEG)
        );

        Pose2d intake1PoseStart = new Pose2d(INTAKE_START_X, INTAKE1_Y, Math.toRadians(180));
        Pose2d intake1PoseEnd = new Pose2d(INTAKE_END_X, INTAKE1_Y, Math.toRadians(180));
        Pose2d gateStart = new Pose2d(gateStart_X, gateStart_Y, Math.toRadians(135));
        Pose2d gateEnd = new Pose2d(gateEnd_X, gateEnd_Y, Math.toRadians(90));

        Pose2d intake2PoseStart = new Pose2d(INTAKE_START_X + INTAKE2_START_OFFSET_X, INTAKE2_Y, Math.toRadians(180));
        Pose2d intake2PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE2_Y, Math.toRadians(180));
        Pose2d dodgeGate = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset + 8, INTAKE2_Y, Math.toRadians(160));

        Pose2d intake3PoseStart = new Pose2d(INTAKE_START_X + INTAKE3_START_OFFSET_X, INTAKE3_Y, Math.toRadians(180));
        Pose2d intake3PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE3_Y, Math.toRadians(180));

        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(180));

        Action toShoot = new ParallelAction(
                drive.actionBuilder(startPose)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading)
                        .build(),

                botActions.actionStartOuttake(SHOOT_RPM),
                botActions.initializeAuto(Indexer.IndexerState.two),

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake),
                        botActions.actionQuickOuttake()
                )
        );

        Action intake1 = botActions.actionIntakeThree(shootingPose, intake1PoseStart, intake1PoseEnd, drive);

        Action backToShoot1 = new ParallelAction(
                drive.actionBuilder(intake1PoseEnd)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake),
                        botActions.actionQuickOuttake()
                )
        );

        Action intake2 = botActions.actionIntakeThree(shootingPose, intake2PoseStart, intake2PoseEnd, drive);

        Action goToGate = drive.actionBuilder(intake2PoseEnd)
                .strafeToSplineHeading(gateStart.position, gateStart.heading)
                .strafeToSplineHeading(gateEnd.position, gateEnd.heading)
                .waitSeconds(gateWaitTime)
                .build();

        Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        .strafeToSplineHeading(dodgeGate.position, dodgeGate.heading)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake + 0.5),
                        botActions.actionQuickOuttake()
                )
        );

        Action intake3 = botActions.actionIntakeThree(shootingPose, intake3PoseStart, intake3PoseEnd, drive);

        Action backToShoot3 = new ParallelAction(
                drive.actionBuilder(intake3PoseEnd)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake + 1.0),
                        botActions.actionQuickOuttake()
                )
        );

        Action toPark = drive.actionBuilder(shootingPose)
                .strafeToSplineHeading(
                        parkPose.position,
                        parkPose.heading
                )
                .build();

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
                        toShoot,
                        /*goToGate,*/
                        intake1,
                        backToShoot1,
                        intake2,
                        backToShoot2,
                        intake3,
                        backToShoot3,
                        toPark
                )
        );

        periodicThread.interrupt();
        try {
            periodicThread.join();
        } catch (InterruptedException ignored) {
        }
    }
}