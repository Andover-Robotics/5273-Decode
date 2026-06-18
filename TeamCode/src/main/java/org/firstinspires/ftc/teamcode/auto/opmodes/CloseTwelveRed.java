package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;
import org.firstinspires.ftc.teamcode.subsystems.Aimer;
import org.firstinspires.ftc.teamcode.teleop.MainTeleopClose;

@Config
@Autonomous(name = "Close Twelve Ball Red Gate No Intake Auto", group = "Autonomous")
public class CloseTwelveRed extends LinearOpMode {
    private Hardware hardware;
    private BotActions botActions;
    private MecanumDrive drive;

    private boolean isFarShooting = false;

    //USE SAME LOCALIZATION STYLE AS AIMER (90 degrees faces the goals, 0 degs faces side with red goal, 180 degs faces side with blue goal, +y is towards goals)
    public static double startX = 113.0;
    public static double startY = 129;
    public static double startAngle = Math.toRadians(90);

    public static double intakingAngle = Math.toRadians(0);

    //row numerations start at 0 for ease
    //array of row y
    public static double[] rowStartY = {83.2, 54.12, 35.0};
    public static double[] rowStartX = {107.0, 102.0, 94.0};
    // array of how far to go forward in each row
    public static double[] rowForwards = {28, 34, 38};

    public static double gateShootOffsetX = -8;
    public static double gateShootOffsetY = -8;

    //shoot pos
    public static double shootY = 83.0;
    public static double shootX = 92.0;

    public static Pose2d startPose = new Pose2d(startX, startY, startAngle);

    // Not necessary, can go directly from shooting pos
    public static Pose2d rowZeroStart = new Pose2d(rowStartX[0], rowStartY[0], intakingAngle);
    public static Pose2d rowOneStart = new Pose2d(rowStartX[1], rowStartY[1], intakingAngle);
    public static Pose2d rowTwoStart = new Pose2d(rowStartX[2], rowStartY[2], intakingAngle);

    //end poses based on rowForwards: +x for red, -x for blue
    public static Pose2d rowZeroEnd = new Pose2d(rowStartX[0] + rowForwards[0], rowStartY[0], intakingAngle);
    public static Pose2d rowOneEnd = new Pose2d(rowStartX[1] + rowForwards[1], rowStartY[1], intakingAngle);
    public static Pose2d rowTwoEnd = new Pose2d(rowStartX[2] + rowForwards[2], rowStartY[2], intakingAngle);

    public static Pose2d gatePose = new Pose2d(131.6, 67, Math.toRadians(-90));
    public static Pose2d shootPos = new Pose2d(shootX, shootY, Math.toRadians(0));
    public static Pose2d leavePos = new Pose2d(103, 72, Math.toRadians(0));



    public Action madeAuto;

    public void makeAuto() {
        drive.localizer.setPose(startPose);

        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

        builder = builder
                .stopAndAdd(botActions.startOuttake(shootPos.position, shootPos.heading.log()))
                .stopAndAdd(botActions.startActions(Aimer.Goal.RED))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(botActions.stopOuttake())

                .strafeToSplineHeading(rowOneStart.position, rowOneStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowOneEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .strafeToSplineHeading(gatePose.position, gatePose.heading.log())
                .stopAndAdd(botActions.startOuttake(new Vector2d(shootPos.position.x + gateShootOffsetX, shootPos.position.y + gateShootOffsetY), shootPos.heading.log()))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(new Vector2d(shootPos.position.x + gateShootOffsetX, shootPos.position.y + gateShootOffsetY), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(botActions.stopOuttake())

                .strafeToSplineHeading(rowTwoStart.position, rowTwoStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowTwoEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake(new Vector2d(shootX - 6, shootY - 6), shootPos.heading.log()))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(new Vector2d(shootX - 6, shootY - 6), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(botActions.stopOuttake())

                //.strafeToSplineHeading(rowZeroStart.position, rowZeroStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowZeroEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake(shootPos.position, shootPos.heading.log()))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(botActions.stopOuttake())

                .strafeToSplineHeading(leavePos.position, leavePos.heading.log());

        madeAuto = builder.build();
    }

    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(hardwareMap, telemetry, startPose, false);
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
            MainTeleopClose.startPose = drive.localizer.getPose();
        }
    }
}
