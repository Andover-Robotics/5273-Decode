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
import org.firstinspires.ftc.teamcode.subsystems.Storage;

@Config
@TeleOp(name = "DistanceRegressionTeleOp", group = "AA_main")
public class DistanceRegressionTeleOp extends LinearOpMode {

    private Intake intake;
    private Storage storage;
    private Outtake outtake;
    private Movement movement;

    private AprilTag aprilTag;
    private Aimer aprilAimer;
    private MecanumDrive drive;

    private long lastAimUpdate = 0;
    private double lastTurnCorrection = 0;
    private double bearingTurnCorrection = 0;

    public static double shooterRPM = 1000;

    private boolean continuousAprilTagLock = false;
    private boolean fieldCentric = false;
    private static final long AIM_UPDATE_INTERVAL_MS = 20;
    private static String colorGoalSelected;

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        storage = new Storage(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        movement = new Movement(hardwareMap, drive);

        aprilTag = new AprilTag(hardwareMap, telemetry);
        aprilAimer = new Aimer(drive);

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
        storage.closeGate();
    }

    public void teleopTick(GamepadEx g1, GamepadEx g2, Telemetry telemetry) {
        drive.updatePoseEstimate();
        outtake.periodic();
        double turnCorrection = 0;

        if (continuousAprilTagLock) {
            long now = System.currentTimeMillis();

            if (now - lastAimUpdate >= AIM_UPDATE_INTERVAL_MS) {
                lastAimUpdate = now;

                double[] data = aprilAimer.calculateLocalizedTurnPower();

                lastTurnCorrection = data[0];
                //shooterRPM = outtake.getRegressionRPM(data[1]);
                bearingTurnCorrection = data[2];
            }

            turnCorrection = lastTurnCorrection;
        } else {
            turnCorrection = 0;
        }

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

        if (g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > .01) {
            intake.run();
        }
        else {
            intake.stop();
        }

        if (g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.01)
            outtake.set(shooterRPM);
        else
            outtake.stop();

        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            storage.openGate();

        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            storage.closeGate();

        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT))
            aprilTag.scanObeliskTag();

        if (g1.wasJustPressed(GamepadKeys.Button.A)) {
            continuousAprilTagLock = true;
            aprilTag.setCurrentCameraScannedId(0);
        }

        if (g1.wasJustPressed(GamepadKeys.Button.B)) {
            continuousAprilTagLock = false;
        }

        if (g1.wasJustPressed(GamepadKeys.Button.X)) {
            if (colorGoalSelected.equals("Blue"))
                aprilAimer.relocalize();
            else if (colorGoalSelected.equals("Red"))
                aprilAimer.relocalize();
            else {
                aprilAimer.relocalize();
            }
        }

        if (g1.wasJustPressed(GamepadKeys.Button.BACK)) {
            aprilTag.setPipeline(0);
            colorGoalSelected = "Blue";
        }

        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            aprilTag.setPipeline(1);
            colorGoalSelected = "Red";
        }

        telemetry.addData("Target RPM", shooterRPM);
        telemetry.addData("Measured RPM", outtake.getRPM());
        telemetry.addData("Turn Correction", turnCorrection);
        telemetry.addData("Heading Error (deg)", bearingTurnCorrection);
        telemetry.addData("April Lock", continuousAprilTagLock);
        telemetry.addData("Alliance", colorGoalSelected);
        telemetry.update();
    }
}
