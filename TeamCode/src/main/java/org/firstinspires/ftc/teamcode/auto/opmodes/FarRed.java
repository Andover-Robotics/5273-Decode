package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Aimer;
import org.firstinspires.ftc.teamcode.teleop.Bot;
import org.firstinspires.ftc.teamcode.teleop.BotPeriodics;
import org.firstinspires.ftc.teamcode.teleop.MainTeleopFar;

@Config
@Autonomous(name = "Far Ball Red Auto", group = "Autonomous")
public class FarRed extends LinearOpMode {
    private Hardware hardware;
    private BotActions botActions;
    private MecanumDrive drive;

    private boolean isFarShooting = true;

    //USE SAME LOCALIZATION STYLE AS AIMER (90 degrees faces the goals, 0 degs faces side with red goal, 180 degs faces side with blue goal, +y is towards goals)
    public static double startY = 6.65;
    public static double startX = 90.2;
    public static double startAngle = Math.toRadians(0);

    public static double intakingAngle = Math.toRadians(0);

    public static double rowStartY = 35;
    public static double rowStartX = 98.0;

    public static double cornerInitialIntakingAngle = -79.5;
    public static double cornerInitialStartY = 21.0;
    public static double cornerInitialStartX = 137.33;

    public static double cornerIntakeEndAngle = 0;
    public static double cornerIntakeEndY = 6.65;
    public static double cornerIntakeEndX = 145.0;

    //shoot pos
    public static double shootY = 16.0;
    public static double shootX = 82.95;

    public static double rowForwards = 42;
    public static double cornerBackwards = 26;


    public static Pose2d startPose = new Pose2d(startX, startY, startAngle);

    public static Pose2d rowStart = new Pose2d(rowStartX, rowStartY, intakingAngle);

    public static Pose2d cornerInitialStart = new Pose2d(cornerInitialStartX, cornerInitialStartY, Math.toRadians(cornerInitialIntakingAngle));

    //end poses based on rowForwards: +x for red, -x for blue
    public static Pose2d rowEnd = new Pose2d(rowStartX + rowForwards, rowStartY, intakingAngle);
    public static Pose2d cornerInitialEnd = new Pose2d(cornerInitialStartX, cornerInitialStartY - cornerBackwards, Math.toRadians(cornerInitialIntakingAngle));

    public static Pose2d cornerIntakeEnd = new Pose2d(cornerIntakeEndX, cornerIntakeEndY, Math.toRadians(cornerIntakeEndAngle));

    public static Pose2d gatePose = new Pose2d(131.6, 67, Math.toRadians(-90));
    public static Pose2d shootPos = new Pose2d(shootX, shootY, Math.toRadians(0));
    public static Pose2d leavePos = new Pose2d(100, 26, Math.toRadians(0));

    public static double cornerIntakeMaxVel = 15;
    public static TranslationalVelConstraint cornerIntakeVelConstraint = new TranslationalVelConstraint(cornerIntakeMaxVel);

    public Action madeAuto;

    public void makeAuto() {
        drive.localizer.setPose(startPose);

        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

        builder = builder
                .stopAndAdd(botActions.actionSetAimlock(true))
                .stopAndAdd(botActions.startActions(Aimer.Goal.RED))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(isFarShooting))
                

                .strafeToSplineHeading(rowStart.position, rowStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                //.strafeToSplineHeading(gatePose.position, gatePose.heading.log())
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(isFarShooting))
                

                .strafeToSplineHeading(cornerInitialStart.position, cornerInitialStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeToSplineHeading(cornerInitialEnd.position, cornerInitialEnd.heading.log(), cornerIntakeVelConstraint)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(isFarShooting))

                // Overflow intake 1
                .stopAndAdd(botActions.startIntake())
                .strafeToSplineHeading(cornerIntakeEnd.position, cornerIntakeEnd.heading.log())
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(isFarShooting))

                // Overflow intake 2
                .stopAndAdd(botActions.startIntake())
                .strafeToSplineHeading(cornerIntakeEnd.position, cornerIntakeEnd.heading.log())
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(isFarShooting))

                // Overflow intake 3
                .stopAndAdd(botActions.startIntake())
                .strafeToSplineHeading(cornerIntakeEnd.position, cornerIntakeEnd.heading.log())
                .stopAndAdd(botActions.runContinuousIntake())
                /* .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(isFarShooting))*/

                .strafeToSplineHeading(leavePos.position, leavePos.heading.log());

        madeAuto = builder.build();
    }

    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(hardwareMap, telemetry, startPose, isFarShooting);
        drive = hardware.mecanumDrive;
        botActions = new BotActions(hardware, telemetry, this);

        while (opModeInInit() && !isStarted() && !isStopRequested()) {
            //temporarily
            telemetry.addData("allicance sleetced", "Red");
            telemetry.addData("yo is the auto bilt gng", madeAuto != null);

            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        if (madeAuto == null) makeAuto();

        try {
            while (opModeIsActive() && !isStopRequested() && madeAuto.run(new com.acmerobotics.dashboard.telemetry.TelemetryPacket())) {
                botActions.actionPeriodic(isFarShooting).run(new com.acmerobotics.dashboard.telemetry.TelemetryPacket());
            }
        }
        finally {
            drive.updatePoseEstimate();
            MainTeleopFar.startPose = drive.localizer.getPose();
            Bot.startPose = drive.localizer.getPose();
            BotPeriodics.goal = Aimer.Goal.RED;
        }
    }
}
