package org.firstinspires.ftc.teamcode.teleop;

import android.telephony.IccOpenLogicalChannelResponse;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
@Config
@TeleOp(name = "DistanceRegressionTeleOp", group = "AA_main")
public class DistanceRegressionTeleOp extends LinearOpMode {

    private Intake intake;
    private Indexer indexer;
    private Actuator actuator;
    private Outtake outtake;
    private Movement movement;

    private AprilTag aprilTag;
    private AprilTagAimer aprilAimer;

    private long lastAimUpdate = 0;
    private double lastTurnCorrection = 0;
    public static int shooterRPM = 5000;

    private boolean continuousAprilTagLock = false;
    private boolean fieldCentric = false;
    private FtcDashboard dash = FtcDashboard.getInstance();

    private static final long AIM_UPDATE_INTERVAL_MS = 50;
    private static int goalTagID;
    private static String colorGoalSelected;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        actuator = new Actuator(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        movement = new Movement(hardwareMap);

        aprilTag = new AprilTag(hardwareMap, telemetry);
        aprilAimer = new AprilTagAimer(hardwareMap);

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        startServos();

        waitForStart();
        while (opModeIsActive()) {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
            gp1.readButtons();
            gp2.readButtons();
            teleopTick(gp1, gp2, telemetry);
        }
    }

    private void startServos() {
        actuator.down();
        indexer.moveTo(Indexer.IndexerState.one);
        indexer.setIntaking(true);
    }

    // teleop type shift
    public void teleopTick(GamepadEx g1, GamepadEx g2, Telemetry telemetry) {
        outtake.periodic();
        //apriltag turn correction
        double turnCorrection = 0;

        if (continuousAprilTagLock) {
            long now = System.currentTimeMillis();

            if (now - lastAimUpdate >= AIM_UPDATE_INTERVAL_MS) {
                lastAimUpdate = now;

                aprilTag.scanGoalTag();
                double bearing = aprilTag.getBearing();

                if (!Double.isNaN(bearing)) {
                    lastTurnCorrection = aprilAimer.calculateTurnPowerFromBearing(bearing);
                } else {
                    lastTurnCorrection = 0;
                    //lastTurnCorrection = aprilAimer.calculateTurnPowerFromBearing(bearing);
                }
            }

            turnCorrection = 0.9 * lastTurnCorrection;  // smooth decay
        }

        //drivetrain control
        if (fieldCentric) {
            movement.teleopTickFieldCentric(
                    g1.getLeftX(),
                    g1.getLeftY(),
                    g1.getRightX(),
                    turnCorrection,
                    true
            );
        } else {
            movement.teleopTick(
                    g1.getLeftX(),
                    g1.getLeftY(),
                    g1.getRightX(),
                    turnCorrection
            );
        }

        // Toggle field centric
        if (g1.getButton(GamepadKeys.Button.LEFT_STICK_BUTTON)) fieldCentric = true;
        if (g1.getButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)) fieldCentric = false;



        // intake control
        if(g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.01){
            intake.run();
        }
        else {
            intake.stop();
        }

        //outtake control
        if (g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.01) {
            outtake.set(shooterRPM);
        } else {
            outtake.stop();
        }

        // spindexer control
        // Advance state
        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            indexer.moveTo(indexer.getState().next());
            telemetry.addLine("Indexer moving");
        }

        //actuator control
        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            actuator.up();
        }
        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            actuator.down();
        }

        // Scan obelisk
        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            aprilTag.scanObeliskTag();
            telemetry.addData("Obelisk ID", aprilTag.getObeliskId());
        }

        // Set intaking ON
        if (g2.wasJustPressed(GamepadKeys.Button.A) && !actuator.isActivated()) {
            indexer.setIntaking(!indexer.isIntaking());
        }

        indexer.update();

        // Begin continuous lock
        if (g2.wasJustPressed(GamepadKeys.Button.X)) {
            continuousAprilTagLock = true;
            aprilTag.setCurrentCameraScannedId(0);
        }

        // Stop continuous lock
        if (g2.wasJustPressed(GamepadKeys.Button.Y)) {
            continuousAprilTagLock = false;
        }

        // Alliance selection
        if (g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            goalTagID = 20;
            aprilTag.setGoalTagID(goalTagID); // blue
            colorGoalSelected = "Blue";
        }

        if (g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            goalTagID = 24;
            aprilTag.setGoalTagID(goalTagID); // red
            colorGoalSelected = "Red";
        }

        // ========== TELEMETRY ==========
        telemetry.addData("Target RPM",outtake.getTargetRPM());
        telemetry.addData("Bot Centerline Range", aprilTag.getBotCenterlineRange());
        telemetry.addData("measured RPM",outtake.getRPM());
        telemetry.addData("Outtake Power", outtake.getPower());
        telemetry.addData("April Lock", continuousAprilTagLock);
        telemetry.addData("Selected Goal Color:", colorGoalSelected);
        telemetry.update();
    }
}
