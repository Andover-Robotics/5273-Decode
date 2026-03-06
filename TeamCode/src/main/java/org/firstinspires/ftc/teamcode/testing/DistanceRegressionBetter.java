package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Pose2d;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Aimer;
import org.firstinspires.ftc.teamcode.teleop.ActionHost;

@Config
@TeleOp(name = "DistanceRegressionBetter", group = "AA_main")
public class DistanceRegressionBetter extends LinearOpMode {

    private Intake intake;
    private Indexer indexer;
    private Actuator actuator;
    private Outtake outtake;
    private Movement movement;

    private AprilTag aprilTag;
    private Aimer aprilAimer;
    private MecanumDrive drive;

    private ActionHost actionHost;

    private long lastAimUpdate = 0;
    private double lastTurnCorrection = 0;
    private double bearingTurnCorrection = 0;

    public static double shooterRPM = 3800;

    private boolean continuousAprilTagLock = false;

    public static double NON_INDEX_SPIN_TIME = 3;
    public static double FULL_BLAST_POWER = 0.25;
    public static double QUICKSPIN_OUTTAKE_RPM_SCALE = 0.94;

    private static final long AIM_UPDATE_INTERVAL_MS = 20;
    private static String colorGoalSelected;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        actuator = new Actuator(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        movement = new Movement(hardwareMap, drive);

        aprilTag = new AprilTag(hardwareMap, telemetry);
        aprilAimer = new Aimer(drive);

        actionHost = new ActionHost();

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        startServos();

        telemetry.addLine("Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            gp1.readButtons();
            gp2.readButtons();
            teleopTick(gp1, gp2);
        }
    }

    private void startServos() {
        actuator.down();
        indexer.moveTo(Indexer.IndexerState.one);
        indexer.setIntaking(true);
    }

    public void teleopTick(GamepadEx g1, GamepadEx g2) {

        drive.updatePoseEstimate();
        outtake.periodic();
        actionHost.update();

        double turnCorrection = 0;

        if (continuousAprilTagLock) {

            long now = System.currentTimeMillis();

            if (now - lastAimUpdate >= AIM_UPDATE_INTERVAL_MS) {
                lastAimUpdate = now;

                double[] data = aprilAimer.calculateLocalizedTurnPower();

                lastTurnCorrection = data[0];
                shooterRPM = outtake.getRegressionRPM(data[1]);
                bearingTurnCorrection = data[2];
            }

            turnCorrection = lastTurnCorrection;
        }

        movement.teleopTick(
                g1.getLeftX(),
                g1.getLeftY(),
                g1.getRightX(),
                turnCorrection
        );

        if (!actionHost.isRunning() && g1.wasJustPressed(GamepadKeys.Button.Y)) {
            actionHost.start(new InstantAction(() -> aprilAimer.relocalize()));
        }

        if (!actionHost.isRunning() && g2.wasJustPressed(GamepadKeys.Button.B)) {
            actionHost.start(actionNonIndexedDump());
        }

        if (g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.01)
            intake.run();
        else
            intake.stop();

        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT))
            indexer.moveTo(indexer.getState().next());

        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            actuator.up();

        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            actuator.down();

        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
            aprilTag.scanObeliskTag();

        if (g2.wasJustPressed(GamepadKeys.Button.A) && !actuator.isActivated())
            indexer.setIntaking(!indexer.isIntaking());

        indexer.update();

        if (g2.wasJustPressed(GamepadKeys.Button.X)) {
            continuousAprilTagLock = !continuousAprilTagLock;
        }

        if (g2.wasJustPressed(GamepadKeys.Button.BACK)) {
            aprilTag.setPipeline(0);
            colorGoalSelected = "Blue";
        }

        if (g2.wasJustPressed(GamepadKeys.Button.START)) {
            aprilTag.setPipeline(1);
            colorGoalSelected = "Red";
        }

        telemetry.addData("Target RPM", shooterRPM);
        telemetry.addData("Measured RPM", outtake.getRPM());
        telemetry.addData("Turn Correction", turnCorrection);
        telemetry.addData("Heading Error (deg)", bearingTurnCorrection);
        telemetry.addData("April Lock", continuousAprilTagLock);
        telemetry.addData("Action Running", actionHost.isRunning());
        telemetry.addData("Alliance", colorGoalSelected);

        telemetry.update();
    }

    private Action actionNonIndexedDump() {

        final double rpm = shooterRPM * QUICKSPIN_OUTTAKE_RPM_SCALE;

        return new SequentialAction(

                new InstantAction(actuator::upQuick),

                new InstantAction(() -> outtake.set(rpm)),

                packet -> !outtake.inRange(100.0),

                new InstantAction(() -> indexer.setIndexerPower(FULL_BLAST_POWER)),

                new SleepAction(NON_INDEX_SPIN_TIME),

                new InstantAction(indexer::stopIndexerPower),
                new InstantAction(outtake::stop),
                new InstantAction(actuator::down),

                new InstantAction(() -> indexer.setIntaking(true)),
                new InstantAction(indexer::initializeColors),
                new InstantAction(() -> indexer.moveTo(Indexer.IndexerState.zero))
        );
    }
}
