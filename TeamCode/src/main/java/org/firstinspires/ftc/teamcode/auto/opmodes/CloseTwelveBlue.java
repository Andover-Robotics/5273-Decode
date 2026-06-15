package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;

@Config
@Autonomous(name = "Close Fifteen Ball Blue Gate No Intake Auto", group = "Autonomous")
public class CloseTwelveBlue extends LinearOpMode {
    private Hardware hardware;
    private BotActions botActions;
    private MecanumDrive drive;

    //USE SAME LOCALIZATION STYLE AS AIMER (90 degrees faces the goals, 0 degs faces side with red goal, 180 degs faces side with blue goal, +y is towards goals)
    public static double startX = 180 - CloseTwelveRed.startX;
    public static double startY = 129;
    public static double startAngle = Math.toRadians(90);
    // based on blue for now : ill make cross compatible soon
    public static double intakingAngle = Math.toRadians(180);

    //row numerations start at 0 for ease
    //array of row y
    public static double[] rowStartY = {81.2, 56.72, 33.0};
    public static double[] rowStartX = {180 - 103.0, 180 - 102.0, 180 - 94.0};
    // array of how far to go forward in each row
    public static double[] rowForwards = {18, 28, 28};

    //shoot pos
    public static double shootY = 81.0;
    public static double shootX = 180 - CloseTwelveRed.shootX;

    public static Pose2d startPose = new Pose2d(startX, startY, startAngle);

    // Not necessary, can go directly from shooting pos
    public static Pose2d rowZeroStart = new Pose2d(rowStartX[0], rowStartY[0], intakingAngle);
    public static Pose2d rowOneStart = new Pose2d(rowStartX[1], rowStartY[1], intakingAngle);
    public static Pose2d rowTwoStart = new Pose2d(rowStartX[2], rowStartY[2], intakingAngle);

    //end poses based on rowForwards: +x for red, -x for blue
    public static Pose2d rowZeroEnd = new Pose2d(rowStartX[0] - rowForwards[0], rowStartY[0], intakingAngle);
    public static Pose2d rowOneEnd = new Pose2d(rowStartX[1] - rowForwards[1], rowStartY[1], intakingAngle);
    public static Pose2d rowTwoEnd = new Pose2d(rowStartX[2] - rowForwards[2], rowStartY[2], intakingAngle);

    public static Pose2d gatePose = new Pose2d(180 - 131.6, 67, Math.toRadians(-90));
    public static Pose2d shootPos = new Pose2d(shootX, shootY, Math.toRadians(180));
    public static Pose2d leavePos = new Pose2d(180 - 103, 76, Math.toRadians(180));

    public static double intakeSettle = 0.25;


    public Action madeAuto;

    public void makeAuto() {
        drive.localizer.setPose(startPose);

        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

        builder = builder
                .stopAndAdd(botActions.startActions())
                .stopAndAdd(new InstantAction(() -> botActions.startOuttake(botActions.getTargetRPM())))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())

                 //.strafeToSplineHeading(rowZeroStart.position, rowZeroStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowZeroEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(new InstantAction(() -> botActions.startOuttake(botActions.getTargetRPM())))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())

                .strafeToSplineHeading(rowOneStart.position, rowOneStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowOneEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .strafeToSplineHeading(gatePose.position, gatePose.heading.log())
                .stopAndAdd(new InstantAction(() -> botActions.startOuttake(botActions.getTargetRPM())))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())

                .strafeToSplineHeading(rowTwoStart.position, rowTwoStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowTwoEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(new InstantAction(() -> botActions.startOuttake(botActions.getTargetRPM())))
                .strafeToSplineHeading(new Vector2d(shootX + 2, shootY + 2), shootPos.heading.log())
                .stopAndAdd(botActions.actionOuttake())
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
    }
}
