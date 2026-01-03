package org.firstinspires.ftc.teamcode.auto.roadrunner.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;


@Config
@Autonomous(name = "RR Standard Strafe Auto", group = "Autonomous")
public class BlueCl extends LinearOpMode {

    // Obelisk
    public static int OBELISK_X = -20;
    public static int OBELISK_Y = 40;
    public static double OBELISK_HEADING_DEG = -120;

    // Shooting
    public static int SHOOT_X = -12;
    public static int SHOOT_Y = 45;
    public static double SHOOT_HEADING_DEG = -45;

    // Intake
    public static int INTAKE_X = -15;

    public static int INTAKE1_Y = 50;
    public static int INTAKE2_Y = 75;
    public static int INTAKE_FORWARD_DIST = 24;

    @Override
    public void runOpMode() {

        // --- Start pose (constructor-only, canonical) ---
        Pose2d startPose = new Pose2d(
                0, 0,
                Math.toRadians(0)
        );

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        // --- Define poses ---
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

        Pose2d intake1Pose = new Pose2d(
                INTAKE_X,
                INTAKE1_Y,
                Math.toRadians(0)
        );

        Pose2d intake1ForwardPose = new Pose2d(
                INTAKE_X + INTAKE_FORWARD_DIST,
                INTAKE1_Y,
                Math.toRadians(0)
        );

        Pose2d intake2Pose = new Pose2d(
                INTAKE_X,
                INTAKE2_Y,
                Math.toRadians(0)
        );

        Pose2d intake2ForwardPose = new Pose2d(
                INTAKE_X + INTAKE_FORWARD_DIST,
                INTAKE2_Y,
                Math.toRadians(0)
        );

        // --- Build actions (standard pattern) ---
        Action toObelisk = drive.actionBuilder(startPose)
                .strafeToLinearHeading(
                        obeliskPose.position,
                        obeliskPose.heading
                )
                .build();

        Action toShoot = drive.actionBuilder(obeliskPose)
                .strafeToLinearHeading(
                        shootingPose.position,
                        shootingPose.heading
                )
                .build();

        Action toIntake1 = drive.actionBuilder(shootingPose)
                .strafeToLinearHeading(
                        intake1Pose.position,
                        intake1Pose.heading
                )
                .build();

        Action intakeForward1 = drive.actionBuilder(intake1Pose)
                .strafeToLinearHeading(
                        intake1ForwardPose.position,
                        intake1ForwardPose.heading
                )
                .build();

        Action backToShoot1 = drive.actionBuilder(intake1ForwardPose)
                .strafeToLinearHeading(
                        shootingPose.position,
                        shootingPose.heading
                )
                .build();

        Action toIntake2 = drive.actionBuilder(shootingPose)
                .strafeToLinearHeading(
                        intake2Pose.position,
                        intake2Pose.heading
                )
                .build();

        Action intakeForward2 = drive.actionBuilder(intake2Pose)
                .strafeToLinearHeading(
                        intake2ForwardPose.position,
                        intake2ForwardPose.heading
                )
                .build();

        Action backToShoot2 = drive.actionBuilder(intake2ForwardPose)
                .strafeToLinearHeading(
                        shootingPose.position,
                        shootingPose.heading
                )
                .build();

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new SequentialAction(
                        toObelisk,
                        toShoot,
                        toIntake1,
                        intakeForward1,
                        backToShoot1,
                        toIntake2,
                        intakeForward2,
                        backToShoot2
                )
        );
    }
}
