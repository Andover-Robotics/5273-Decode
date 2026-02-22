package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.ConcreteLazyImu;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Movement;

import org.openftc.easyopencv.*;

@TeleOp(name = "AprilTagTester", group = "testing")
public class AprilTagTester extends LinearOpMode {
    private Intake intake;
    private Indexer indexer;
    private Actuator actuator;
    private Outtake outtake;
    private Movement movement;

    private AprilTag aprilTag;
    private AprilTagAimer aprilAimer;
    private MecanumDrive drive;

    private long currentTime = 0;
    private long lastTick = 0;
    private long lastAimUpdateTime = 0;
    private double lastTurnCorrection = 0;
    public static double shooterRPM;

    private boolean continuousGoalLock = false;
    private boolean fieldCentric = false;

    public static long aimUpdateInterval = 20; // ms
    private static String colorGoalSelected;
    private static Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        actuator = new Actuator(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        drive = new MecanumDrive(hardwareMap, startPose);
        movement = new Movement(hardwareMap, drive);

        aprilTag = new AprilTag(hardwareMap, telemetry);
        aprilAimer = new AprilTagAimer(hardwareMap, drive);

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
        //drive.updatePoseEstimate();

        double turnCorrection = 0;
        if (continuousGoalLock) {
            currentTime = System.currentTimeMillis();

            // Run scan + PID only every AIM_UPDATE_INTERVAL_MS
            if (currentTime - lastAimUpdateTime >= aimUpdateInterval) {
                lastAimUpdateTime = currentTime;

                aprilTag.scanGoalTag();
                double bearing = aprilTag.getBearing();

                if (!Double.isNaN(bearing)) {
                    turnCorrection = aprilAimer.calculateTurnPowerFromBearing(bearing);
                }
            }

            // turnCorrection = 0.9 * lastTurnCorrection; - don't want this
        } else {
            turnCorrection = 0;
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
        if(g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)>0.01){
            intake.run();
        }
        else {
            intake.stop();
        }

        //outtake control
        if (g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.01) {
            outtake.set(shooterRPM);
        } else {
            outtake.stop();
        }

        // spindexer control
        // Advance state
        if (g1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            indexer.moveTo(indexer.getState().next());
        }

        //actuator control
        if (g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)) {
            actuator.up();
        }
        if (g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            actuator.down();
        }

        // Scan obelisk
        if (g1.wasJustPressed(GamepadKeys.Button.DPAD_LEFT)) {
            aprilTag.scanObeliskTag();
        }

        // Set intaking ON
        if (g1.wasJustPressed(GamepadKeys.Button.A) && !actuator.isActivated()) {
            indexer.setIntaking(!indexer.isIntaking());
        }

        if (g1.wasJustPressed(GamepadKeys.Button.B)) {
            movement.setPose(new Pose2d(0, 0, Math.toRadians(0)));
        }

        indexer.update();

        // Begin continuous lock
        if (g1.wasJustPressed(GamepadKeys.Button.X)) {
            continuousGoalLock = true;
            aprilTag.setCurrentCameraScannedId(0);
        }

        // Stop continuous lock
        if (g1.wasJustPressed(GamepadKeys.Button.Y)) {
            continuousGoalLock = false;
        }

        // Alliance selection
        if (g1.wasJustPressed(GamepadKeys.Button.BACK)) {
            aprilTag.setPipeline(0);
            colorGoalSelected = "Blue";
        }

        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            aprilTag.setPipeline(1);
            colorGoalSelected = "Red";
        }

        // ========== TELEMETRY ==========
        /*telemetry.addData("A: Set intaking, B: Reset Localized Pose, X/Y: lock in/unlock, Dpad left: Scan Obelisk Id, Dpad Up/Down: actuator, Dpad Right:", "index");
        telemetry.addData("Target RPM",outtake.getTargetRPM());
        telemetry.addData("Bot Range", aprilTag.getRange()); // moved limelight
        telemetry.addData("measured RPM",outtake.getRPM());
        telemetry.addData("Outtake Power", outtake.getPower());
        telemetry.addData("Localized Lock", continuousGoalLock);
        //telemetry.addData("x", drive.localizer.getPose().position.x);
        //telemetry.addData("y", drive.localizer.getPose().position.y);
        //telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.log()));
        telemetry.addData("Selected Goal Color:", colorGoalSelected);
        telemetry.addData("Selected Goal Color:", colorGoalSelected);
        telemetry.addData("Obelisk ID", aprilTag.getObeliskId());*/
        telemetry.addData("Loop time: ", currentTime - lastTick);
        telemetry.update();
        lastTick = currentTime;
    }
}


