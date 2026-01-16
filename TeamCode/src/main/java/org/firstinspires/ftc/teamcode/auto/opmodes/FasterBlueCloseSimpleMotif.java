package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
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
@Autonomous(name = "Faster Simple Blue Auto With Motif", group = "Autonomous")
public class FasterBlueCloseSimpleMotif extends LinearOpMode {

    public static double OBELISK_X = -12;
    public static double OBELISK_Y = 38;
    public static double OBELISK_HEADING_DEG = -120;

    public static double SHOOT_X = -12;
    public static double SHOOT_Y = 42;
    public static double SHOOT_HEADING_DEG = -45;

    public static double INTAKE_X = -10;
    public static double INTAKE1_Y = 51;
    public static double INTAKE2_Y = 75;
    public static double INTAKE_FORWARD_DIST = 8;

    public static double PARK_X = -6;
    public static double PARK_Y = 68;

    public static int SHOOT_RPM = 4010;
    public static int timeToStartOuttakeBeforeToOuttake = 1;

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

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        Pose2d obeliskPose = new Pose2d(OBELISK_X, OBELISK_Y, Math.toRadians(OBELISK_HEADING_DEG));
        Pose2d shootingPose = new Pose2d(SHOOT_X, SHOOT_Y, Math.toRadians(SHOOT_HEADING_DEG));

        Pose2d intake1PoseStart = new Pose2d(INTAKE_X, INTAKE1_Y, Math.toRadians(0));
        Pose2d intake1Pose3 = new Pose2d(INTAKE_X + 3 * INTAKE_FORWARD_DIST + 3, INTAKE1_Y, Math.toRadians(0));

        Pose2d intake2PoseStart = new Pose2d(INTAKE_X, INTAKE2_Y, Math.toRadians(0));
        Pose2d intake2Pose3 = new Pose2d(INTAKE_X + 3 * INTAKE_FORWARD_DIST + 3.5, INTAKE2_Y, Math.toRadians(0));
        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(0));

        Action toObelisk = drive.actionBuilder(startPose)
                .strafeToLinearHeading(obeliskPose.position, obeliskPose.heading)
                .stopAndAdd(botActions.actionScanObelisk())
                .build();

        Action toShoot = new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(startPose)
                                .strafeToLinearHeading(shootingPose.position, shootingPose.heading)
                                .build(),

                        new SequentialAction(
                                new SleepAction(0), // no need
                                botActions.actionQuickOuttake(SHOOT_RPM)
                        )
                )
        );

        Action toIntakeStart1 = drive.actionBuilder(shootingPose)
                .strafeToLinearHeading(intake1PoseStart.position, intake1PoseStart.heading)
                .build();

        Action toIntake1_3 = new ParallelAction(
                drive.actionBuilder(intake1PoseStart)
                        .strafeToLinearHeading(intake1Pose3.position, intake1Pose3.heading)
                        .build(),
                botActions.actionIntakeThreeFast()
        );

        Action backToShoot1 = new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(intake1Pose3)
                                .strafeToLinearHeading(shootingPose.position, shootingPose.heading)
                                .build(),

                        new SequentialAction(
                                new SleepAction(timeToStartOuttakeBeforeToOuttake), // just waits
                                botActions.actionQuickOuttake(SHOOT_RPM)
                        )
                )
        );

        Action toIntakeStart2 = drive.actionBuilder(shootingPose)
                .strafeToLinearHeading(intake2PoseStart.position, intake2PoseStart.heading)
                .build();

        Action toIntake2_3 = new ParallelAction(
                drive.actionBuilder(intake1PoseStart)
                        .strafeToLinearHeading(intake1Pose3.position, intake1Pose3.heading)
                        .build(),
                botActions.actionIntakeThreeFast()
        );

        Action backToShoot2 = new SequentialAction(
                new ParallelAction(
                        drive.actionBuilder(intake2Pose3)
                                .strafeTo(new Vector2d(INTAKE_X - 3 * INTAKE_FORWARD_DIST, INTAKE2_Y))
                                .strafeToLinearHeading(shootingPose.position, shootingPose.heading)
                                .build(),

                        new SequentialAction(
                                new SleepAction(timeToStartOuttakeBeforeToOuttake), // just waits
                                botActions.actionQuickOuttake(SHOOT_RPM)
                        )
                )
        );

        Action toPark = drive.actionBuilder(startPose)
                .strafeToLinearHeading(
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
                        toIntakeStart1,
                        toIntake1_3,
                        backToShoot1,
                        toIntakeStart2,
                        toIntake2_3,
                        backToShoot2,
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
