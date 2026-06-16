package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
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

@Config
@Autonomous(name = "Close Fifteen Ball Red Gate Intake Auto", group = "Autonomous")
public class CloseFifteenRedGateIntake extends LinearOpMode {
    private Hardware hardware;
    private BotActions botActions;
    private MecanumDrive drive;

    //USE SAME LOCALIZATION STYLE AS AIMER (90 degrees faces the goals, 0 degs faces side with red goal, 180 degs faces side with blue goal, +y is towards goals)
    public static double startX = CloseTwelveRed.startX;
    public static double startY = CloseTwelveRed.startY;
    public static double startAngle = Math.toRadians(90);

    public static double intakingAngle = Math.toRadians(0);

    //row numerations start at 0 for ease
    //array of row y
    public static double[] rowStartY = CloseTwelveRed.rowStartY;
    public static double[] rowStartX = CloseTwelveRed.rowStartX;
    // array of how far to go forward in each row
    public static double[] rowForwards = CloseTwelveRed.rowForwards;

    //shoot pos
    public static double shootY = CloseTwelveRed.shootY;
    public static double shootX = CloseTwelveRed.shootX;
    public static double secondShootRowOffsetX = 6;
    public static double secondShootRowOffsetY = 6;

    public static double gatePoseStartY = 52.85;
    public static double gatePoseStartX = 120.85;
    public static double gatePoseEndY = 59.0;
    public static double gatePoseEndX = 130;

    public static double gateAngle = 30.0;

    public static Pose2d startPose = new Pose2d(startX, startY, startAngle);

    // Not necessary, can go directly from shooting pos
    public static Pose2d rowZeroStart = new Pose2d(rowStartX[0], rowStartY[0], intakingAngle);
    public static Pose2d rowOneStart = new Pose2d(rowStartX[1], rowStartY[1], intakingAngle);
    public static Pose2d rowTwoStart = new Pose2d(rowStartX[2], rowStartY[2], intakingAngle);

    //end poses based on rowForwards: +x for red, -x for blue
    public static Pose2d rowZeroEnd = new Pose2d(rowStartX[0] + rowForwards[0], rowStartY[0], intakingAngle);
    public static Pose2d rowOneEnd = new Pose2d(rowStartX[1] + rowForwards[1], rowStartY[1], intakingAngle);
    public static Pose2d rowTwoEnd = new Pose2d(rowStartX[2] + rowForwards[2], rowStartY[2], intakingAngle);

    public static Pose2d gatePoseStart = new Pose2d(gatePoseStartX, gatePoseStartY, Math.toRadians(gateAngle));
    public static Pose2d gatePoseEnd = new Pose2d(gatePoseEndX, gatePoseEndY, Math.toRadians(gateAngle));

    public static Pose2d shootPos = new Pose2d(shootX, shootY, Math.toRadians(0));
    public static Pose2d leavePos = CloseTwelveRed.leavePos;

    public static double intakeSettle = 0.25;
    public static double gateWaitSeconds = 1.6;


    public Action madeAuto;

    public void makeAuto() {
        drive.localizer.setPose(startPose);

        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

        builder = builder
                // preload
                .stopAndAdd(botActions.startActions(Aimer.Goal.RED))
                .stopAndAdd(botActions.startOuttake())
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
                .stopAndAdd(botActions.stopOuttake())

                 // row 0
                 //.strafeToSplineHeading(rowZeroStart.position, rowZeroStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowZeroEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake())
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
                .stopAndAdd(botActions.stopOuttake())

                // row 1
                .strafeToSplineHeading(rowOneStart.position, rowOneStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowOneEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake())
                .strafeToSplineHeading(new Vector2d(shootPos.position.x - secondShootRowOffsetX, shootPos.position.y - secondShootRowOffsetY), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
                .stopAndAdd(botActions.stopOuttake())

                // Gate intake 1
                .strafeToSplineHeading(gatePoseStart.position, gatePoseStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeToSplineHeading(gatePoseEnd.position, gatePoseEnd.heading.log())
                .waitSeconds(gateWaitSeconds)
                .stopAndAdd(botActions.runContinuousIntake())

                .stopAndAdd(botActions.startOuttake())
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
                .stopAndAdd(botActions.stopOuttake())

                // Gate intake 2
                .strafeToSplineHeading(gatePoseStart.position, gatePoseStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeToSplineHeading(gatePoseEnd.position, gatePoseEnd.heading.log())
                .waitSeconds(gateWaitSeconds)
                .stopAndAdd(botActions.runContinuousIntake())

                .stopAndAdd(botActions.startOuttake())
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
                .stopAndAdd(botActions.stopOuttake())

                /*
                // row 2
                .strafeToSplineHeading(rowTwoStart.position, rowTwoStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowTwoEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake())
                .strafeToSplineHeading(new Vector2d(shootX - 2, shootY + 2), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
                .stopAndAdd(botActions.stopOuttake())
                .strafeToSplineHeading(leavePos.position, leavePos.heading.log())*/;

        madeAuto = builder.build();
    }

    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(hardwareMap, telemetry, startPose);
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

        Actions.runBlocking(new ParallelAction(
                botActions.actionPeriodic(),
                madeAuto
        ));
    }
}
