package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SleepAction;
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
import org.firstinspires.ftc.teamcode.teleop.MainTeleop;

@Config
@Autonomous(name = "Close Fifteen Ball Blue Gate Intake Auto", group = "Autonomous")
public class CloseFifteenBlueGateIntake extends LinearOpMode {
    private Hardware hardware;
    private BotActions botActions;
    private MecanumDrive drive;

    //USE SAME LOCALIZATION STYLE AS AIMER (90 degrees faces the goals, 0 degs faces side with red goal, 180 degs faces side with blue goal, +y is towards goals)
    public static double startX = CloseTwelveBlue.startX;
    public static double startY = CloseTwelveBlue.startY;
    public static double startAngle = Math.toRadians(90);

    public static double intakingAngle = Math.toRadians(180);

    //row numerations start at 0 for ease
    //array of row y
    public static double[] rowStartY = CloseTwelveBlue.rowStartY;
    public static double[] rowStartX = CloseTwelveBlue.rowStartX;
    // array of how far to go forward in each row
    public static double[] rowForwards = CloseTwelveBlue.rowForwards;

    //shoot pos
    public static double shootY = CloseTwelveBlue.shootY;
    public static double shootX = CloseTwelveBlue.shootX;
    public static double secondShootRowOffsetX = -CloseFifteenRedGateIntake.secondShootRowOffsetX;
    public static double secondShootRowOffsetY = CloseFifteenRedGateIntake.secondShootRowOffsetY;
    public static double gateShootOffsetX = -CloseFifteenRedGateIntake.gateShootOffsetX;
    public static double gateShootOffsetY = CloseFifteenRedGateIntake.gateShootOffsetY;

    public static double gatePoseStartY = CloseFifteenRedGateIntake.gatePoseStartY;
    public static double gatePoseStartX = 144 - CloseFifteenRedGateIntake.gatePoseStartX;
    public static double gatePoseEndY = CloseFifteenRedGateIntake.gatePoseEndY;
    public static double gatePoseEndX = 144 - CloseFifteenRedGateIntake.gatePoseEndX;

    public static double gate2YOffset = 0.5;


    public static double gateAngle = 180 - CloseFifteenRedGateIntake.gateAngle;

    public static Pose2d startPose = new Pose2d(startX, startY, startAngle);

    // Not necessary, can go directly from shooting pos
    public static Pose2d rowZeroStart = new Pose2d(rowStartX[0], rowStartY[0], intakingAngle);
    public static Pose2d rowOneStart = new Pose2d(rowStartX[1], rowStartY[1], intakingAngle);
    public static Pose2d rowTwoStart = new Pose2d(rowStartX[2], rowStartY[2], intakingAngle);

    //end poses based on rowForwards: +x for red, -x for blue
    public static Pose2d rowZeroEnd = new Pose2d(rowStartX[0] - rowForwards[0], rowStartY[0], intakingAngle);
    public static Pose2d rowOneEnd = new Pose2d(rowStartX[1] - rowForwards[1], rowStartY[1], intakingAngle);
    public static Pose2d rowTwoEnd = new Pose2d(rowStartX[2] - rowForwards[2], rowStartY[2], intakingAngle);

    public static Pose2d gatePoseStart = new Pose2d(gatePoseStartX, gatePoseStartY, Math.toRadians(gateAngle));
    public static Pose2d gatePoseEnd = new Pose2d(gatePoseEndX, gatePoseEndY, Math.toRadians(gateAngle));

    public static Pose2d shootPos = new Pose2d(shootX, shootY, Math.toRadians(180));
    public static Pose2d leavePos = CloseTwelveBlue.leavePos;

    public static double intakeSettle = 0.25;
    public static double gateWaitSeconds = CloseFifteenRedGateIntake.gateWaitSeconds;


    public Action madeAuto;

    public void makeAuto() {
        drive.localizer.setPose(startPose);

        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

        builder = builder
                // preload
                .stopAndAdd(botActions.startActions(Aimer.Goal.BLUE))
                .stopAndAdd(botActions.startOuttake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
                .stopAndAdd(botActions.stopOuttake())

                // row 1
                .strafeToSplineHeading(rowOneStart.position, rowOneStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowOneEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(new Vector2d(shootPos.position.x + secondShootRowOffsetX, shootPos.position.y + secondShootRowOffsetY), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
                .stopAndAdd(botActions.stopOuttake())

                // Gate intake 1
                .strafeToSplineHeading(gatePoseStart.position, gatePoseStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeToSplineHeading(gatePoseEnd.position, gatePoseEnd.heading.log())
                .waitSeconds(gateWaitSeconds)
                .stopAndAdd(botActions.runContinuousIntake())

                .stopAndAdd(botActions.startOuttake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(new Vector2d(shootPos.position.x + gateShootOffsetX, shootPos.position.y + gateShootOffsetY), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
                .stopAndAdd(botActions.stopOuttake())

                // Gate intake 2
                .strafeToSplineHeading(gatePoseStart.position, gatePoseStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeToSplineHeading(new Vector2d(gatePoseEndX - gate2YOffset, gatePoseEndY - gate2YOffset), gatePoseEnd.heading.log())
                .waitSeconds(gateWaitSeconds)
                .stopAndAdd(botActions.runContinuousIntake())

                .stopAndAdd(botActions.startOuttake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(new Vector2d(shootPos.position.x + gateShootOffsetX, shootPos.position.y + gateShootOffsetY), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
                .stopAndAdd(botActions.stopOuttake())

                // row 0
                //.strafeToSplineHeading(rowZeroStart.position, rowZeroStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowZeroEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake())
                .stopAndAdd(botActions.actionSetAimlock(true))
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
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(new Vector2d(shootX + 2, shootY + 2), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
                .stopAndAdd(botActions.stopOuttake())
                 */
                .strafeToSplineHeading(leavePos.position, leavePos.heading.log());

        madeAuto = builder.build();
    }

    public void runOpMode() throws InterruptedException {
        hardware = new Hardware(hardwareMap, telemetry, startPose);
        drive = hardware.mecanumDrive;
        botActions = new BotActions(hardware, telemetry, this);

        while (opModeInInit() && !isStarted() && !isStopRequested()) {
            //temporarily
            telemetry.addData("allicance sleetced", "Blue");
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
        MainTeleop.startPose = drive.localizer.getPose();
    }
}
