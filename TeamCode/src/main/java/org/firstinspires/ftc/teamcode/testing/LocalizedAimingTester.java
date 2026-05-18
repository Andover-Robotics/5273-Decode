package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.Aimer;

@Config
@TeleOp(name = "Localized aiming tester", group = "AA_main")
public class LocalizedAimingTester extends LinearOpMode {

    private Intake intake;
    private Indexer indexer;
    private Actuator actuator;
    private Outtake outtake;
    private Movement movement;

    private AprilTag aprilTag;
    private Aimer aimer;
    private MecanumDrive drive;

    private long currentTime = 0;
    private long lastTick = 0;
    private long lastAimUpdateTime = 0;
    private double lastTurnCorrection = 0;
    private double bearingTurnCorrection = 0;
    public static double shooterRPM;

    private boolean continuousGoalLock = false;
    private boolean fieldCentric = false;

    public static long aimUpdateInterval = 20; // ms
    private static String colorGoalSelected = "";
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
        aimer = new Aimer(drive);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        startServos();

        waitForStart();
        while (opModeIsActive()) {
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
        drive.updatePoseEstimate();

        double turnCorrection = 0;
        double[] data = {0, 0, 0};
        if (continuousGoalLock) {
            currentTime = System.currentTimeMillis();

            // Run scan + PID only every AIM_UPDATE_INTERVAL_MS
            if (currentTime - lastAimUpdateTime >= aimUpdateInterval) {
                lastAimUpdateTime = currentTime;
                data = aimer.calculateLocalizedTurnPower();
                lastTurnCorrection = data[0];
                shooterRPM = outtake.getRegressionRPM(data[1]);
                bearingTurnCorrection = data[2];
            }

            turnCorrection = lastTurnCorrection;

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
        if (g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.01) {
            intake.run();
        } else {
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
            if (colorGoalSelected.equals("Blue"))
                aimer.relocalize();
            else if (colorGoalSelected.equals("Red"))
                aimer.relocalize();
            else {
                aimer.relocalize();
            }
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
            aimer.setBlueTarget();
            colorGoalSelected = "Blue";
        }

        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            aprilTag.setPipeline(1);
            aimer.setRedTarget();
            colorGoalSelected = "Red";
        }

        // ========== TELEMETRY ==========
        telemetry.addData("A: Set intaking, B: Reset Localized Pose, X/Y: lock in/unlock, Dpad left: Scan Obelisk Id, Dpad Up/Down: actuator, Dpad Right:", "index");
        telemetry.addData("Target RPM", outtake.getTargetRPM());
        telemetry.addData("Bot Range", data[1]); // moved limelight
        telemetry.addData("measured RPM", outtake.getRPM());
        telemetry.addData("Outtake Power", outtake.getPower());
        telemetry.addData("Localized Lock", continuousGoalLock);
        telemetry.addData("x", drive.localizer.getPose().position.x);
        telemetry.addData("y", drive.localizer.getPose().position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.log()));
        telemetry.addData("heading error (deg)", bearingTurnCorrection);
        telemetry.addData("tagPose x", Aimer.tagPose.position.x);
        telemetry.addData("tagPose y", Aimer.tagPose.position.y);
        telemetry.addData("Selected Goal Color:", colorGoalSelected);
        telemetry.addData("Obelisk ID", aprilTag.getObeliskId());
        long now = System.currentTimeMillis();
        telemetry.addData("Loop time: ", now - lastTick);
        telemetry.update();
        lastTick = now;
        currentTime = now;
    }
}

