package org.firstinspires.ftc.teamcode.auto.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
@Autonomous(name = "Close Fifteen Ball Red Gate Intake SOTM Auto", group = "Autonomous")
public class CloseFifteenRedGateIntakeSOTM extends LinearOpMode {
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
    public static double shootYStart = 92.0;
    public static double shootXStart = 113.0;
    public static double shootYEnd = 83.0;
    public static double shootXEnd = 92.0;
    
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

    public static Pose2d shootPosStart = new Pose2d(shootXStart, shootYStart, Math.toRadians(0));
    public static Pose2d shootPosEnd = new Pose2d(shootXEnd, shootYEnd, Math.toRadians(0));
    public static Pose2d leavePos = CloseTwelveRed.leavePos;

    public static double gateWaitSeconds = 1.6;

    public static double SOTMMaxVel = 10;
    public static TranslationalVelConstraint SOTMVelConstraint = new TranslationalVelConstraint(SOTMMaxVel);

    public Action madeAuto;

    public void makeAuto() {
        drive.localizer.setPose(startPose);

        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);

        builder = builder
                // preload
                .stopAndAdd(botActions.startActions(Aimer.Goal.RED))
                .stopAndAdd(botActions.startOuttake(shootPosStart.position, shootPosStart.heading.log()))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(shootPosStart.position, shootPosStart.heading.log())
                .stopAndAdd(SOTM(drive, new Pose2d(shootPosStart.position, shootPosStart.heading.log()), shootPosEnd))
                .stopAndAdd(botActions.stopOuttake())

                // row 1
                .strafeToSplineHeading(rowOneStart.position, rowOneStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowOneEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake(shootPosStart.position, shootPosStart.heading.log()))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(new Vector2d(shootPosStart.position.x + 20, shootPosStart.position.y), shootPosStart.heading.log())
                .strafeToSplineHeading(shootPosStart.position, shootPosStart.heading.log())
                .stopAndAdd(SOTM(drive, new Pose2d(shootPosStart.position, shootPosStart.heading.log()), shootPosEnd))
                .stopAndAdd(botActions.stopOuttake())

                // Gate intake 1
                .strafeToSplineHeading(gatePoseStart.position, gatePoseStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeToSplineHeading(gatePoseEnd.position, gatePoseEnd.heading.log())
                .waitSeconds(gateWaitSeconds)
                .stopAndAdd(botActions.runContinuousIntake())

                .stopAndAdd(botActions.startOuttake(shootPosStart.position, shootPosStart.heading.log()))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(shootPosStart.position, shootPosStart.heading.log())
                .stopAndAdd(SOTM(drive, new Pose2d(shootPosStart.position, shootPosStart.heading.log()), shootPosEnd))
                .stopAndAdd(botActions.stopOuttake())

                // Gate intake 2
                .strafeToSplineHeading(gatePoseStart.position, gatePoseStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeToSplineHeading(new Vector2d(gatePoseEndX - gate2YOffset, gatePoseEndY - gate2YOffset), gatePoseEnd.heading.log())
                .waitSeconds(gateWaitSeconds)
                .stopAndAdd(botActions.runContinuousIntake())

                .stopAndAdd(botActions.startOuttake(shootPosStart.position, shootPosStart.heading.log()))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(shootPosStart.position, shootPosStart.heading.log())
                .stopAndAdd(SOTM(drive, new Pose2d(shootPosStart.position, shootPosStart.heading.log()), shootPosEnd))
                .stopAndAdd(botActions.stopOuttake())

                /*
                // Gate intake 3shootPosStart.position, shootPosStart.heading.log()
                .strafeToSplineHeading(gatePoseStart.position, gatePoseStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeToSplineHeading(new Vector2d(gatePoseEndX - gate2YOffset, gatePoseEndY - gate2YOfshootPosStart.position, shootPosStart.heading.log()fset), gatePoseEnd.heading.log())
                .waitSeconds(gateWaitSeconds)
                .stopAndAdd(botActions.runContinuousIntake())

                .stopAndAdd(botActions.startOuttake(shootPosStart.position, shootPosStart.heading.log()))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(new Vector2d(shootPosStart.position.x + gateShootOffsetX, shootPosStart.position.y + gateShootOffsetY), shootPosStart.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(botActions.stopOuttake())
                */

                // row 0
                //.strafeToSplineHeading(rowZeroStart.position, rowZeroStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowZeroEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake(new Vector2d(shootPosStart.position.x + 15, shootPosStart.position.y + 18), shootPosStart.heading.log()))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(new Vector2d(shootPosStart.position.x + 15, shootPosStart.position.y + 18), shootPosStart.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(SOTM(drive, new Pose2d(new Vector2d(shootPosStart.position.x + 15, shootPosStart.position.y + 18), shootPosStart.heading.log()), new Pose2d(new Vector2d(shootPosEnd.position.x + 15, shootPosEnd.position.y + 18), shootPosEnd.heading.log())));

                /*
                // row 2
                .strafeToSplineHeading(rowTwoStart.position, rowTwoStart.heading.log())
                .stopAndAdd(botActions.startIntake())
                .strafeTo(rowTwoEnd.position)
                .stopAndAdd(botActions.runContinuousIntake())
                .stopAndAdd(botActions.startOuttake(shootPosStart.position, shootPosStart.heading.log()))
                .stopAndAdd(botActions.actionSetAimlock(true))
                .strafeToSplineHeading(new Vector2d(shootX - 2, shootY + 2), shootPosStart.heading.log())
                .stopAndAdd(botActions.actionOuttake(false))
                .stopAndAdd(botActions.stopOuttake())
                */
                //.strafeToSplineHeading(leavePos.position, leavePos.heading.log());

        madeAuto = builder.build();
    }

    public Action SOTM(MecanumDrive drive, Pose2d startPose, Pose2d shootEnd) {
        return new ParallelAction(
                drive.actionBuilder(startPose)
                        .strafeToSplineHeading(new Vector2d(shootEnd.position.x, shootEnd.position.y), shootEnd.heading.log(), SOTMVelConstraint)
                        .build(),

                new InstantAction(() -> botActions.actionOuttake(isFarShooting))
        );
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
