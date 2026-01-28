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
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@Config
@Autonomous(name = "Faster Simple Blue Auto With Motif using Disp Intaking", group = "Autonomous")
public class FasterBlueCloseSimpleMotifDisp extends LinearOpMode {

    public static double OBELISK_X = -8;
    public static double OBELISK_Y = 36;
    public static double OBELISK_HEADING_DEG = -120;

    public static double SHOOT_X = -12;
    public static double SHOOT_Y = 42;
    public static double SHOOT_HEADING_DEG = -50;

    public static double INTAKE_X = -8;

    public static double gate_Y = 63;
    public static double INTAKE1_Y = 51;
    public static double INTAKE2_Y = 75;
    public static double INTAKE3_Y = 99;
    public static double INTAKE_FORWARD_DIST = 8;

    public static double PARK_X = -6;
    public static double PARK_Y = 68;

    public static int SHOOT_RPM = 4010;

    public static double timeUntilStartOuttake = 1.0; // Time until you start the outtake action, which still includes the spinup time

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

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

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

        Pose2d intake1PoseStart = new Pose2d(INTAKE_X, INTAKE1_Y, Math.toRadians(0));
        Pose2d intake1PoseEnd = new Pose2d(INTAKE_X + 3 * INTAKE_FORWARD_DIST + 4, INTAKE1_Y, Math.toRadians(0));
        Pose2d gate = new Pose2d(INTAKE_X + 3 * INTAKE_FORWARD_DIST + 4, gate_Y, Math.toRadians(-90));

        Pose2d intake2PoseStart = new Pose2d(INTAKE_X, INTAKE2_Y, Math.toRadians(0));
        Pose2d intake2PoseEnd = new Pose2d(INTAKE_X + 3 * INTAKE_FORWARD_DIST + 4.5, INTAKE2_Y, Math.toRadians(0));
        Pose2d dodgeGate = new Pose2d(INTAKE_X + 2 * INTAKE_FORWARD_DIST, INTAKE2_Y - 2, Math.toRadians(-20));

        Pose2d intake3PoseStart = new Pose2d(INTAKE_X, INTAKE3_Y, Math.toRadians(0));
        Pose2d intake3PoseEnd = new Pose2d(INTAKE_X + 3 * INTAKE_FORWARD_DIST + 4.5, INTAKE3_Y, Math.toRadians(0));

        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(0));

        Action toObelisk = drive.actionBuilder(startPose)
                .strafeToSplineHeading(obeliskPose.position, obeliskPose.heading)
                .stopAndAdd(botActions.actionScanObelisk())
                .build();

        Action toShoot = new ParallelAction(
                drive.actionBuilder(obeliskPose)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading)
                        .build(),/*

                new SequentialAction(
                        botActions.rotateToMotifColorBeforeOuttake(0, botActions.getObeliskId(), 0),
                        new SleepAction(timeUntilStartOuttake),
                        botActions.actionQuickOuttake(SHOOT_RPM)
                )
                */
                new InstantAction(() -> botActions.initializeForIntake(Indexer.IndexerState.two)) // only temporary for testing, this is done in actionQuickOuttake
        );

        Action intake1 = botActions.actionIntakeThreeUsingDisp(shootingPose, intake1PoseStart, intake1PoseEnd, drive);

        Action backToShoot1 = new ParallelAction(
                drive.actionBuilder(intake1PoseEnd)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading)
                        .build(),/*

                new SequentialAction(
                        botActions.rotateToMotifColorBeforeOuttake(1, botActions.getObeliskId(), 0),
                        new SleepAction(timeUntilStartOuttake),
                        botActions.actionQuickOuttake(SHOOT_RPM)
                )
                */
                new InstantAction(() -> botActions.initializeForIntake(Indexer.IndexerState.two)) // only temporary for testing, this is done in actionQuickOuttake
        );

        Action intake2 = botActions.actionIntakeThreeUsingDisp(shootingPose, intake2PoseStart, intake2PoseEnd, drive);

        Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        .strafeToSplineHeading(dodgeGate.position, dodgeGate.heading)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading)
                        .build(),/*

                new SequentialAction(
                        botActions.rotateToMotifColorBeforeOuttake(2, botActions.getObeliskId(), 0),
                        new SleepAction(timeUntilStartOuttake),
                        botActions.actionQuickOuttake(SHOOT_RPM)
                )
                */
                new InstantAction(() -> botActions.initializeForIntake(Indexer.IndexerState.two)) // only temporary for testing, this is done in actionQuickOuttake
        );

        Action intake3 = botActions.actionIntakeThreeUsingDisp(shootingPose, intake3PoseStart, intake3PoseEnd, drive);

        Action backToShoot3 = new ParallelAction(
                drive.actionBuilder(intake3PoseEnd)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading)
                        .build(),/*

                new SequentialAction(
                        botActions.rotateToMotifColorBeforeOuttake(3, botActions.getObeliskId(), 0),
                        new SleepAction(timeUntilStartOuttake),
                        botActions.actionQuickOuttake(SHOOT_RPM)
                )
                */
                new InstantAction(() -> botActions.initializeForIntake(Indexer.IndexerState.two)) // only temporary for testing, this is done in actionQuickOuttake
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
                        toObelisk,
                        toShoot,
                        /*gate,*/
                        intake1,
                        backToShoot1,
                        intake2,
                        backToShoot2,
                        /*
                        intake3,
                        backToShoot3,
                         */
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