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
import org.firstinspires.ftc.teamcode.teleop.MainTeleopClose;

@Config
@Autonomous(name = "Close Fifteen Ball Red Gate Intake Auto With Splines", group = "Autonomous")
public class CloseFifteenRedGateIntakeSpline extends LinearOpMode {
    private Hardware hardware;
    private BotActions botActions;
    private MecanumDrive drive;

    private boolean isFarShooting = false;

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
    public static double secondShootRowOffsetX = -7;
    public static double secondShootRowOffsetY = -7;
    public static double gateShootOffsetX = -10;
    public static double gateShootOffsetY = -10;

    public static double gatePoseStartY = 58.5;
    public static double gatePoseStartX = 132.5;
    public static double gatePoseEndY = 59.5;
    public static double gatePoseEndX = 133.5;

    public static double gate2YOffset = 0.0;

    public static double gateAngle = 28.0;

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

    public static double gateWaitSeconds = 1.6;


    public Action madeAuto;

    public void makeAuto() {
        drive.localizer.setPose(startPose);

        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

        builder = builder
                // --- PRELOAD ---
                .stopAndAdd(botActions.startOuttake())
                .stopAndAdd(botActions.startActions(Aimer.Goal.RED))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .splineToLinearHeading(shootPos, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(botActions.stopOuttake())

                // --- ROW 1 (Merged Intake) ---
                // Drive directly to the end of the row, starting the intake 3 inches into the move
                .splineToLinearHeading(rowOneEnd, rowOneEnd.heading.log())
                .afterDisp(3.0, botActions.startIntake())

                // Once at the end of the row, prep the outtake and curve back to shoot
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                .splineToLinearHeading(new Pose2d(new Vector2d(shootPos.position.x + secondShootRowOffsetX, shootPos.position.y + secondShootRowOffsetY), shootPos.heading.log()), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(botActions.stopOuttake())

                // --- GATE INTAKE 1 (Merged Intake) ---
                .splineToLinearHeading(gatePoseEnd, gatePoseEnd.heading.log())
                .afterDisp(2.0, botActions.startIntake())
                .waitSeconds(gateWaitSeconds) // Necessary pause to grab game elements

                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                .splineToLinearHeading(new Pose2d(new Vector2d(shootPos.position.x + gateShootOffsetX, shootPos.position.y + gateShootOffsetY), shootPos.heading.log()), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(botActions.stopOuttake())

                // --- GATE INTAKE 2 (Merged Intake) ---
                .splineToLinearHeading(new Pose2d(new Vector2d(gatePoseEndX - gate2YOffset, gatePoseEndY - gate2YOffset), gatePoseEnd.heading.log()), gatePoseEnd.heading.log())
                .afterDisp(2.0, botActions.startIntake())
                .waitSeconds(gateWaitSeconds) // Necessary pause

                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                .splineToLinearHeading(new Pose2d(new Vector2d(shootPos.position.x + gateShootOffsetX, shootPos.position.y + gateShootOffsetY), shootPos.heading.log()), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(botActions.stopOuttake())

                // --- ROW 0 (Merged Intake) ---
                .splineToLinearHeading(rowZeroEnd, rowZeroEnd.heading.log())
                .afterDisp(3.0, botActions.startIntake())

                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake())
                .stopAndAdd(botActions.actionSetAimlock(true))
                .splineToLinearHeading(new Pose2d(new Vector2d(shootPos.position.x + 15, shootPos.position.y + 18), shootPos.heading.log()), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(botActions.stopOuttake());

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
