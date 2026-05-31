package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Aimer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Movement;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

@Config
public class BotPeriodics {
    protected final Intake intake;
    protected final Outtake outtake;
    protected final Movement movement;
    protected final Aimer aimer;
    protected final MecanumDrive drive;
    protected final Storage storage;
    protected final Turret turret;
    protected final GamepadEx g1;
    protected final GamepadEx g2;
    protected final Telemetry telemetry;
    protected double bearingTurnCorrection = 0;
    private double bearingAvoidCorrection = 0;
    public static double BEARING_AVOID_IN_DEGREES = 20;
    protected ActionHost actionHost;
    // camera vision
    protected boolean fieldCentric = false;
    public static boolean drivetrainAim = false;
    protected boolean aimLock = false;
    protected long lastAimUpdate = 0;
    protected double lastTurnCorrection = 0.0;
    protected double turnCorrection = 0.0;
    protected String colorGoalSelected = "";
    protected boolean rangeRequested = false;
    protected double[] targetData = {0,0,0};

    protected boolean continuousIntake = false;

    public static double targetRPM = 2000;
    protected static final long AIM_UPDATE_INTERVAL_MS = 20;

    protected boolean twoMovementMode;

    public BotPeriodics(HardwareMap hardwareMap, Telemetry tele, MecanumDrive mecanumDrive, Gamepad gamepad1, Gamepad gamepad2, boolean useMovement) {
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        drive = mecanumDrive;
        storage = new Storage(hardwareMap);
        turret = new Turret(hardwareMap);
        movement = new Movement(hardwareMap, drive);
        aimer = new Aimer(drive);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        actionHost = new ActionHost();
        telemetry = tele;
        twoMovementMode = useMovement;
    }
    
    protected void handlePeriodics()
    {
        g1.readButtons();
        g2.readButtons();

        handleIntake();
        handleAimLock();
        handleMovement();
        handleAllianceSelection();
        handleTelemetry();

        outtake.periodic();
        actionHost.update();
        drive.updatePoseEstimate();

        if (outtake.getTargetRPM() > 0) {
            outtake.set(targetRPM);
        }

        if(rangeRequested || aimLock){
            long now = System.currentTimeMillis();

            if (now - lastAimUpdate >= AIM_UPDATE_INTERVAL_MS) {
                lastAimUpdate = now;
                targetData = aimer.calculateLocalizedTurnPower();
                lastTurnCorrection = targetData[0];
                targetRPM = outtake.getRegressionRPM(targetData[1]);
                bearingTurnCorrection = targetData[2];
            }
            turnCorrection = lastTurnCorrection;
        }
    }
    private void handleIntake() {
        if(g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) continuousIntake = !continuousIntake;
        double leftTrigger = g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double leftTrigger2 = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double rightTrigger2 = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        boolean leftDown = leftTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE || leftTrigger2 > TeleopConstants.Gamepad.TRIGGER_DEADZONE;
        boolean rightDown = rightTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE || rightTrigger2 > TeleopConstants.Gamepad.TRIGGER_DEADZONE;
        if(leftDown) intake.run();
        else if(rightDown) intake.runBackwards();
        else if(continuousIntake) intake.runSlow();
        else intake.stop();
    }

    // Periodic Handlers
    protected void handleTelemetry()
    {
        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Outtake RPM", outtake.getRPM());
        telemetry.addData("Target RMP", outtake.getTargetRPM());
        telemetry.addData("Bot Range", targetData[1]);
        telemetry.addData("Alliance selected", colorGoalSelected);
        telemetry.addData("Turn Correction:", turnCorrection);
        telemetry.addData("Intake power: ", intake.getPower());
        telemetry.addData("Last Turn Correction", lastTurnCorrection);
        telemetry.update();
    }

    protected void handleAllianceSelection() {
        if (g1.wasJustPressed(GamepadKeys.Button.BACK)) {
            aimer.setBlueTarget();
            colorGoalSelected = "Blue";
        }
        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            aimer.setRedTarget();
            colorGoalSelected = "Red";
        }
    }

    protected void handleMovement() {
        double lx = g1.getLeftX();
        double ly = g1.getLeftY();
        double rx = g1.getRightX();
        if(twoMovementMode){
             lx = g2.getLeftX();
             ly = g2.getLeftY();
             rx = g2.getRightX();
        }

        if (drivetrainAim) {
            if (fieldCentric) {
                movement.teleopTickFieldCentric(
                        lx,
                        ly,
                        rx,
                        turnCorrection,
                        true
                );
            } else {
                movement.teleopTick(
                        lx,
                        ly,
                        rx,
                        turnCorrection
                );
            }
        }
        else {
            if (bearingTurnCorrection >= 180 - BEARING_AVOID_IN_DEGREES){
                bearingAvoidCorrection = aimer.calculateTurnPowerFromBearing(-BEARING_AVOID_IN_DEGREES);
            }
            if (bearingTurnCorrection <= -180 + BEARING_AVOID_IN_DEGREES) {
                bearingAvoidCorrection = aimer.calculateTurnPowerFromBearing(BEARING_AVOID_IN_DEGREES);
            }

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

            bearingAvoidCorrection = 0;

            turret.rotate(Math.toDegrees(drive.localizer.getPose().heading.log()));
        }
    }

    protected void handleAimLock() {
        // Toggle continuous lock
        if (g1.wasJustPressed(GamepadKeys.Button.A)) {
            aimLock = true;
            g1.gamepad.rumbleBlips(2);
        }
        if (g1.wasJustPressed(GamepadKeys.Button.B)) {
            aimLock = false;
            g1.gamepad.rumbleBlips(1);
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
        if (!aimLock){
            turnCorrection = 0;
        }
    }

    protected double getTargetRPM() {
        return targetRPM;
    }
}

