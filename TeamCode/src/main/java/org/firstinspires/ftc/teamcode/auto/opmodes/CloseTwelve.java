package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.utils.BotActions;
import org.firstinspires.ftc.teamcode.auto.utils.Hardware;

@Config
@Autonomous(name = "Close Twelve Ball Blue Auto", group = "Autonomous")
public class CloseTwelve extends LinearOpMode {
    private final Hardware hardware = new Hardware(hardwareMap, telemetry, startPose);
    private final BotActions botActions = new BotActions(hardware, telemetry, this);
    private final MecanumDrive drive = hardware.mecanumDrive;

    //USE SAME LOCALIZATION STYLE AS AIMER (90 degrees faces the goals, 0 degs faces side with red goal, 180 degs faces side with blue goal, +y is towards goals)
    public static double startX = 114.367;
    public static double startY = 131.988;
    public static double startAngle = Math.toRadians(90);
    // based on blue for now : ill make cross compatible soon
    public static double intakingAngle = Math.toRadians(0);

    //row numerations start at 0 for ease
    //array of row y, placeholder value for now
    public static double[] rowStartY = {83.3, 59.2, 34.9};
    public static double[] rowStartX = {104.9, 104.15, 104.6};
    // array of how far to go forward in each row, placeholder value for now
    public static double[] rowForwards = {10, 10, 10};

    //shoot pos
    public static double shootX = 101.233;
    public static double shootY = 88.6;

    public static Pose2d startPose = new Pose2d(startX, startY, startAngle);
    public static Pose2d rowZeroStart = new Pose2d(rowStartX[0], rowStartY[0], intakingAngle);
    public static Pose2d rowOneStart = new Pose2d(rowStartX[1], rowStartY[1], intakingAngle);
    public static Pose2d rowTwoStart = new Pose2d(rowStartX[2], rowStartY[2], intakingAngle);

    //end poses based on rowForwards: +x for red, -x for blue
    public static Pose2d rowZeroEnd = new Pose2d(rowStartX[0] - rowForwards[0], rowStartY[0], intakingAngle);
    public static Pose2d rowOneEnd = new Pose2d(rowStartX[1] - rowForwards[1], rowStartY[1], intakingAngle);
    public static Pose2d rowTwoEnd = new Pose2d(rowStartX[2] - rowForwards[2], rowStartY[2], intakingAngle);

    public static Pose2d gatePose = new Pose2d(131.6, 67, Math.toRadians(-90));
    public static Pose2d shootPos = new Pose2d(shootX, shootY, Math.toRadians(0));
    public static Pose2d leavePos = new Pose2d(101, 78, Math.toRadians(0));

    public static double intakeSettle = 0.25;
    public static double startAimlockTimeBeforeShoot = 0.5;


    public Action madeAuto;

    public void makeAuto() {
        drive.localizer.setPose(startPose);

        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

        builder = builder
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(outtakeAction(drive))
                .strafeToSplineHeading(rowZeroStart.position, rowZeroStart.heading.log())
                .stopAndAdd(new InstantAction(botActions::startIntake))
                .strafeTo(rowZeroEnd.position)
                .stopAndAdd(new InstantAction(botActions::runContinuousIntake))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(outtakeAction(drive))
                .strafeToSplineHeading(rowOneStart.position, rowOneStart.heading.log())
                .stopAndAdd(new InstantAction(botActions::startIntake))
                .strafeTo(rowOneEnd.position)
                .stopAndAdd(new InstantAction(botActions::runContinuousIntake))
                .strafeToSplineHeading(gatePose.position, gatePose.heading.log())
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(outtakeAction(drive))
                .strafeToSplineHeading(rowTwoStart.position, rowTwoStart.heading.log())
                .stopAndAdd(new InstantAction(botActions::startIntake))
                .strafeTo(rowTwoEnd.position)
                .stopAndAdd(new InstantAction(botActions::runContinuousIntake))
                .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                .stopAndAdd(outtakeAction(drive))
                .strafeToSplineHeading(leavePos.position, leavePos.heading.log());

        madeAuto = builder.build();
    }

    Action outtakeAction(MecanumDrive drive) {
        return new ParallelAction(
                drive.actionBuilder(startPose)
                        .strafeToSplineHeading(shootPos.position, shootPos.heading.log())
                        .build(),

                botActions.startOuttake(botActions.getTargetRPM()),

                new SequentialAction(
                        botActions.actionSetAimlock(true),
                        new SleepAction(startAimlockTimeBeforeShoot),
                        botActions.actionOuttake()
                )
        );
    }


    public void runOpMode() throws InterruptedException {
        GamepadEx gp1 = new GamepadEx(gamepad1);

        while (opModeInInit() && !isStarted() && !isStopRequested()) {
            gp1.readButtons();
            //temporarily
            telemetry.addData("allicance sleetced", "Blue");
            telemetry.addData("yo is the auto bilt gng", madeAuto != null);

            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
                makeAuto();
            }

            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;

        if (madeAuto == null) makeAuto();

        Actions.runBlocking(new ParallelAction(
                packet -> {
                    botActions.actionPeriodic();
                    return true;
                },
                madeAuto
        ));
    }
}
