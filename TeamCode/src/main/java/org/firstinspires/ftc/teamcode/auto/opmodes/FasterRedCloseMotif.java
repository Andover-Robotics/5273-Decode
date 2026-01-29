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
@Autonomous(name = "Faster Red Auto With Motif", group = "Autonomous")
public class FasterRedCloseMotif extends LinearOpMode {

    public static double OBELISK_X = 8;
    public static double OBELISK_Y = 36;
    public static double OBELISK_HEADING_DEG = -60;

    public static double SHOOT_X = 15;
    public static double SHOOT_Y = 46;
    public static double SHOOT_HEADING_DEG = -134;
    public static double SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT = 6;

    public static double INTAKE_X = 12;
    public static double INTAKE2_OFFSET_X = 2;
    public static double INTAKE3_OFFSET_X = 4;

    public static double INTAKE_END_X = -18;

    public static double gate_X = -2;
    public static double gate_Y = 63;

    public static double INTAKE1_Y = 52;
    public static double INTAKE2_Y = 77;
    public static double INTAKE3_Y = 101;
    public static double intake2And3_XIncrease = 2.0;

    public static double PARK_X = 6;
    public static double PARK_Y = 68;

    public static int SHOOT_RPM = 3480;

    public static double timeUntilStartOuttake = 0.65; // Time until you start the outtake action, which still includes the spinup time

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
        Pose2d intake1PoseEnd = new Pose2d(INTAKE_END_X, INTAKE1_Y, Math.toRadians(180));
        Pose2d gate = new Pose2d(gate_X, gate_Y, Math.toRadians(-90));

        Pose2d intake2PoseStart = new Pose2d(INTAKE_X + INTAKE2_OFFSET_X, INTAKE2_Y, Math.toRadians(180));
        Pose2d intake2PoseEnd = new Pose2d(INTAKE_END_X - intake2And3_XIncrease, INTAKE2_Y, Math.toRadians(180));
        Pose2d dodgeGate = new Pose2d(INTAKE_END_X - intake2And3_XIncrease + 8, INTAKE2_Y - 2, Math.toRadians(-160));

        Pose2d intake3PoseStart = new Pose2d(INTAKE_X + INTAKE3_OFFSET_X, INTAKE3_Y, Math.toRadians(180));
        Pose2d intake3PoseEnd = new Pose2d(INTAKE_END_X - intake2And3_XIncrease, INTAKE3_Y, Math.toRadians(180));

        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(180));

        // Later combine this with toShoot for smoother
        Action toObelisk = new ParallelAction(
                drive.actionBuilder(startPose)
                        .strafeToSplineHeading(obeliskPose.position, obeliskPose.heading)
                        .build(),

                botActions.initializeAuto(Indexer.IndexerState.two),
                botActions.actionStartOuttake(SHOOT_RPM)
            // .stopAndAdd(botActions.actionScanObelisk()) - Gotta update from Quali-2 for the pipeline
        );

        Action toShoot = new ParallelAction(
                drive.actionBuilder(obeliskPose)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading)
                        .build(),

                new SequentialAction(
                        //botActions.rotateToMotifColorBeforeOuttake(0, botActions.getObeliskId(), 0), - no need in 12 ball or more
                        new SleepAction(timeUntilStartOuttake),
                        botActions.actionQuickOuttake()
                )

                //new InstantAction(() -> botActions.initializeForIntake(Indexer.IndexerState.two)) // only temporary for testing, this is done in actionQuickOuttake
        );

        Action intake1 = botActions.actionIntakeThree(shootingPose, intake1PoseStart, intake1PoseEnd, drive);

        Action backToShoot1 = new ParallelAction(
                drive.actionBuilder(intake1PoseEnd)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                new SequentialAction(
                        botActions.actionStartOuttake(SHOOT_RPM),
                        //botActions.rotateToMotifColorBeforeOuttake(1, botActions.getObeliskId(), 0),
                        new SleepAction(timeUntilStartOuttake),
                        botActions.actionQuickOuttake()
                )

                //new InstantAction(() -> botActions.initializeForIntake(Indexer.IndexerState.two)) // only temporary for testing, this is done in actionQuickOuttake
        );

        Action intake2 = botActions.actionIntakeThree(shootingPose, intake2PoseStart, intake2PoseEnd, drive);

        Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        .strafeToSplineHeading(dodgeGate.position, dodgeGate.heading)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                new SequentialAction(
                        botActions.actionStartOuttake(SHOOT_RPM),
                        //botActions.rotateToMotifColorBeforeOuttake(2, botActions.getObeliskId(), 0),
                        new SleepAction(timeUntilStartOuttake + 0.5),
                        botActions.actionQuickOuttake()
                )

                //new InstantAction(() -> botActions.initializeForIntake(Indexer.IndexerState.two)) // only temporary for testing, this is done in actionQuickOuttake
        );

        Action intake3 = botActions.actionIntakeThree(shootingPose, intake3PoseStart, intake3PoseEnd, drive);

        Action backToShoot3 = new ParallelAction(
                drive.actionBuilder(intake3PoseEnd)
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                new SequentialAction(
                        botActions.actionStartOuttake(SHOOT_RPM),
                        //botActions.rotateToMotifColorBeforeOuttake(3, botActions.getObeliskId(), 0),
                        new SleepAction(timeUntilStartOuttake + 1.0),
                        botActions.actionQuickOuttake()
                )

                //new InstantAction(() -> botActions.initializeForIntake(Indexer.IndexerState.two)) // only temporary for testing, this is done in actionQuickOuttake
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
