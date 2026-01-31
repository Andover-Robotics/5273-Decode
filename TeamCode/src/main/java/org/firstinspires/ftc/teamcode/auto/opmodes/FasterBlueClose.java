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
@Autonomous(name = "Faster Blue Auto No Motif", group = "Autonomous")
public class FasterBlueClose extends LinearOpMode {

    public static double maxIntakeDrivingVel = 17;

    public static double SHOOT_X = -17;
    public static double SHOOT_Y = 40;
    public static double SHOOT_HEADING_DEG = -50;
    public static double SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT = 0; // -2

    public static double INTAKE_START_X = -12;
    public static double INTAKE2_START_OFFSET_X = -3.0;
    public static double INTAKE3_START_OFFSET_X = -5.0;
    public static double INTAKE_END_X = 16;
    public static double intake_END_2And3_XOffset = -6.0;

    public static double INTAKE1_Y = 50;
    public static double INTAKE2_Y = 76.5;
    public static double INTAKE3_Y = 96;

    public static double gate_X = 12;
    public static double gate_Y = 60;

    public static double gateWaitTime = 1.5;

    public static double PARK_X = -6;
    public static double PARK_Y = 68;

    public static int SHOOT_RPM = 3240;

    public static double timeUntilStartOuttake = 1.65; // Time until you start the outtake action, which still includes the wait for actuator

    // has quick outtake and quick intake
    @Override
    public void runOpMode() {
        Hardware hardware = new Hardware(hardwareMap, telemetry, this);
        BotActions botActions = hardware.actions;
        MecanumDrive drive = hardware.mecanumDrive;

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

        Pose2d shootingPose = new Pose2d(
                SHOOT_X,
                SHOOT_Y,
                Math.toRadians(SHOOT_HEADING_DEG)
        );

        Pose2d intake1PoseStart = new Pose2d(INTAKE_START_X, INTAKE1_Y, Math.toRadians(0));
        Pose2d intake1PoseEnd = new Pose2d(INTAKE_END_X, INTAKE1_Y, Math.toRadians(0));
        Pose2d gate = new Pose2d(gate_X, gate_Y, Math.toRadians(-90));

        Pose2d intake2PoseStart = new Pose2d(INTAKE_START_X + INTAKE2_START_OFFSET_X, INTAKE2_Y, Math.toRadians(0));
        Pose2d intake2PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE2_Y, Math.toRadians(0));
        Pose2d dodgeGate = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset - 14, INTAKE2_Y - 4, Math.toRadians(40));

        Pose2d intake3PoseStart = new Pose2d(INTAKE_START_X + INTAKE3_START_OFFSET_X, INTAKE3_Y, Math.toRadians(0));
        Pose2d intake3PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE3_Y, Math.toRadians(0));

        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(0));

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

        Action intake1 = botActions.actionIntakeThreeFeedback(shootingPose, intake1PoseStart, intake1PoseEnd, drive, maxIntakeDrivingVel);

        Action goToGate = drive.actionBuilder(intake1PoseEnd)
                .strafeToLinearHeading(gate.position, gate.heading)
                .waitSeconds(gateWaitTime)
                .build();

        Action backToShoot1 = new ParallelAction(
                drive.actionBuilder(intake1PoseEnd) // goToGate
                        .strafeToSplineHeading(shootingPose.position, shootingPose.heading.plus(Math.toRadians(SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT)))
                        .build(),

                botActions.actionStartOuttake(SHOOT_RPM),

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake),
                        botActions.actionQuickOuttake()
                )
        );

        Action intake2 = botActions.actionIntakeThreeFeedback(shootingPose, intake2PoseStart, intake2PoseEnd, drive, maxIntakeDrivingVel);

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

        /*Action backToShoot2 = new ParallelAction(
                drive.actionBuilder(intake2PoseEnd)
                        .setTangent(Math.toRadians(0))

                        .splineToSplineHeading(
                                shootingPose,
                                Math.toRadians(160)
                        )
                        .build(),

                botActions.actionStartOuttake(SHOOT_RPM),
                botActions.rotateToMotifColorBeforeOuttake(2, botActions::getObeliskId, 0),

                new SequentialAction(
                        new SleepAction(timeUntilStartOuttake + 0.5),
                        botActions.actionQuickOuttake()
                )
        );*/

        Action intake3 = botActions.actionIntakeThreeFeedback(shootingPose, intake3PoseStart, intake3PoseEnd, drive, maxIntakeDrivingVel);

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

        Actions.runBlocking(
                new ParallelAction(
                        botActions.actionPeriodic(),
                        new SequentialAction(
                                toShoot,
                                intake1,
                                /*goToGate,*/
                                backToShoot1,
                                intake2,
                                backToShoot2,
                                intake3,
                                backToShoot3,
                                toPark
                        )
                )
        );
    }
}