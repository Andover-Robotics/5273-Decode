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
@TeleOp(name = "Localized aiming tester", group = "AA_main")
public class LocalizedAimingTester extends LinearOpMode {

    private Intake intake;
    private Storage storage;
    private Outtake outtake;
    private Movement movement;
    private Turret turret;

    private AprilTag aprilTag;
    private Aimer aimer;
    private MecanumDrive drive;

    private long currentTime = 0;
    private long lastTick = 0;
    private long lastAimUpdateTime = 0;
    private double lastTurnCorrection = 0;
    private double bearingTurnCorrection = 0;
    private double bearingAvoidCorrection = 0;
    public static double goalHeading = 0;
    public static double BEARING_AVOID_IN_DEGREES = 30;
    public static boolean mecanumAvoid = true;
    public static double servoOffset = 86.0;
    public static double shooterRPM;

    public static double theAngle1 = 0;
    public static double theAngle2 = 0;

    private boolean aimlock = false;
    private boolean fieldCentric = false;
    public static boolean drivetrainAim = false;

    public static long aimUpdateInterval = 20; // ms
    private static String colorGoalSelected = "";
    private static Pose2d startPose = new Pose2d(0, 0, Math.toRadians(0));

    @Override
    public void runOpMode() throws InterruptedException {
        intake = new Intake(hardwareMap);
        storage = new Storage(hardwareMap, intake);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        drive = new MecanumDrive(hardwareMap, startPose);
        movement = new Movement(hardwareMap, drive);
        turret = new Turret(hardwareMap);

        aprilTag = new AprilTag(hardwareMap, telemetry);
        aimer = new Aimer(drive);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        GamepadEx gp1 = new GamepadEx(gamepad1);
        GamepadEx gp2 = new GamepadEx(gamepad2);

        startServos();
        storage.closeGate();

        waitForStart();
        while (opModeIsActive()) {
            gp1.readButtons();
            gp2.readButtons();
            teleopTick(gp1, gp2, telemetry);
        }
    }

    private void startServos() {
        storage.closeGate();
    }

    // wrapped to (-180, 180)
    private double angleWrapDegrees(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    // teleop type shift
    public void teleopTick(GamepadEx g1, GamepadEx g2, Telemetry telemetry) {
        outtake.periodic();
        drive.updatePoseEstimate();
        storage.updateForIfFull();

        double turnCorrection = 0;
        double[] data = {0, 0, 0};
        currentTime = System.currentTimeMillis();

        // Run scan + PID only every AIM_UPDATE_INTERVAL_MS
        if (currentTime - lastAimUpdateTime >= aimUpdateInterval) {
            lastAimUpdateTime = currentTime;
            data = aimer.calculateLocalizedData();
            lastTurnCorrection = data[0];
            shooterRPM = outtake.getRegressionRPM(data[1]);
            bearingTurnCorrection = data[2];
            goalHeading = data[3];
        }

        turnCorrection = lastTurnCorrection;
        // turnCorrection = 0.9 * lastTurnCorrection; - don't want this

        if (aimlock) {
            //drivetrain control
            if (drivetrainAim) {
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
            } else {
                if (mecanumAvoid) {
                    // Avoiding the heading where servo must wraparound, to disable set BEARING_AVOID_IN_DEGREES = 0
                    double wrappedTurnCorrection = angleWrapDegrees(bearingTurnCorrection + servoOffset + 180);
                    double posLimit = 180 - BEARING_AVOID_IN_DEGREES; // counter clockwise limit
                    double negLimit = -180 + BEARING_AVOID_IN_DEGREES; // clockwise limit

                    if (wrappedTurnCorrection >= posLimit) {
                        bearingAvoidCorrection = aimer.calculateTurnPowerFromBearing(-(wrappedTurnCorrection - posLimit));
                    } else if (wrappedTurnCorrection <= negLimit) {
                        bearingAvoidCorrection = aimer.calculateTurnPowerFromBearing(negLimit - wrappedTurnCorrection);
                    } else {
                        bearingAvoidCorrection = 0;
                    }
                }
                else {
                    bearingAvoidCorrection = 0;
                }

                // turret control
                if (fieldCentric) {
                    movement.teleopTickFieldCentric(
                            g1.getLeftX(),
                            g1.getLeftY(),
                            g1.getRightX(),
                            bearingAvoidCorrection,
                            true
                    );
                } else {
                    movement.teleopTick(
                            g1.getLeftX(),
                            g1.getLeftY(),
                            g1.getRightX(),
                            bearingAvoidCorrection
                    );
                }

                turret.rotate(bearingTurnCorrection);
            }
        }
        else {
            // No lock in
            if (fieldCentric) {
                movement.teleopTickFieldCentric(
                        g1.getLeftX(),
                        g1.getLeftY(),
                        g1.getRightX(),
                        0,
                        true
                );
            } else {
                movement.teleopTick(
                        g1.getLeftX(),
                        g1.getLeftY(),
                        g1.getRightX(),
                        0
                );
            }
        }

        // Toggle field centric
        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) fieldCentric = true;
        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) fieldCentric = false;


        // intake control
        if (g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.01) {
            intake.run();
            storage.runTransfer();
        }
        else if (g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.01) {
            intake.run();
            storage.runTransfer();
        }
        else if (g1.getButton(GamepadKeys.Button.A))
            intake.run();
        else if (g1.getButton(GamepadKeys.Button.B))
            storage.runTransfer();
        else {
            intake.stop();
            storage.stopTransfer();
        }

        /* Right is used for outtake for testing
        // Eject
        else if (g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.01) {
            intake.runBackwards();
            storage.runTransferBackwards();
        }
        else {
            intake.stop();
            storage.stopTransfer();
        }*/

        if (g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.01)
            outtake.set(shooterRPM);
        else if (g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.01) // Testing
            outtake.set(shooterRPM);
        else
            outtake.stop();

        if (g1.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            storage.openGate();

        if (g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            storage.closeGate();


        if (g1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            turret.setServos(0);
        }

        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            aimlock = true;
            g1.gamepad.rumbleBlips(2);
            g2.gamepad.rumbleBlips(2);
            aprilTag.setCurrentCameraScannedId(0);
        }

        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            aimlock = false;
            g1.gamepad.rumbleBlips(1);
            g2.gamepad.rumbleBlips(1);
        }

        if (g1.wasJustPressed(GamepadKeys.Button.X)) {
            if (colorGoalSelected.equals("Blue"))
                aimer.relocalize();
            else if (colorGoalSelected.equals("Red"))
                aimer.relocalize();
            else {
                aimer.relocalize();
            }
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
        telemetry.addData("Localized Lock", aimlock);
        telemetry.addData("x", drive.localizer.getPose().position.x);
        telemetry.addData("y", drive.localizer.getPose().position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.localizer.getPose().heading.log()));
        telemetry.addData("Goal heading (deg)", goalHeading);
        telemetry.addData("heading error (deg)", bearingTurnCorrection);
        telemetry.addData("tagPose x", Aimer.tagPose.position.x);
        telemetry.addData("tagPose y", Aimer.tagPose.position.y);
        telemetry.addData("Selected Goal Color:", colorGoalSelected);
        telemetry.addData("Obelisk ID", aprilTag.getObeliskId());
        long now = System.currentTimeMillis();
        telemetry.addData("Loop time: ", now - lastTick);
        telemetry.addData("Transfer Current (amps): ", storage.getCurrent());
        telemetry.update();
        lastTick = now;
        currentTime = now;
    }
}

