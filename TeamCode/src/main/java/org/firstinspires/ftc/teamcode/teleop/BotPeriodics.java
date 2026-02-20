package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.ConcreteLazyImu;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Movement;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Config
public class BotPeriodics {
    protected final Intake intake;
    protected final Indexer indexer;
    protected final Actuator actuator;
    protected final Outtake outtake;
    protected final Movement movement;
    protected final AprilTag aprilTag;
    protected final AprilTagAimer aprilAimer;
    protected final MecanumDrive drive;

    protected final GamepadEx g1;
    protected final GamepadEx g2;
    protected final Telemetry telemetry;

    protected ActionHost actionHost;

    // camera vision
    protected boolean fieldCentric = false;
    protected boolean continuousAprilTagLock = false;
    protected long lastAimUpdate = 0;
    protected double lastTurnCorrection = 0.0;
    protected double turnCorrection = 0.0;
    protected int goalTagID;
    protected String colorGoalSelected;

    protected boolean continuousIntake = true;

    public static double targetRPM = 3800;
    protected static final long AIM_UPDATE_INTERVAL_MS = 20;

    protected boolean twoMovementMode;

    public static double rangeOffset = 6.67;

    public BotPeriodics(HardwareMap hardwareMap, Telemetry tele, MecanumDrive mecanumDrive, Gamepad gamepad1, Gamepad gamepad2, boolean useMovement) {
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        actuator = new Actuator(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        drive = mecanumDrive;
        movement = new Movement(hardwareMap, drive);
        aprilTag = new AprilTag(hardwareMap, tele);
        aprilAimer = new AprilTagAimer(hardwareMap, drive);
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

        if(g2.wasJustPressed(GamepadKeys.Button.DPAD_DOWN))
            continuousIntake = !continuousIntake;

        double leftTrigger = g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double leftTrigger2 = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        double rightTrigger2 = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        boolean leftDown = leftTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE || leftTrigger2 > TeleopConstants.Gamepad.TRIGGER_DEADZONE;
        boolean rightDown = rightTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE || rightTrigger2 > TeleopConstants.Gamepad.TRIGGER_DEADZONE;

        boolean inRange = indexer.isWithinTargetDegrees(5);

        if(leftDown){
            if (inRange) intake.run();
            else intake.runSlow();
        }
        else if(rightDown) intake.runBackwards();
        else if(continuousIntake) intake.runSlow();
        else intake.stop();

        // driver one (constant
        handleAprilTagLock();
        handleMovement();
        handleAllianceSelection();

        handleTelemetry();

        indexer.update();
        outtake.periodic();
        actionHost.update();
        drive.updatePoseEstimate();
    }

    // Periodic Handlers

    protected void handleTelemetry()
    {
        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Indexer State", "%s -> %s",
                indexer.getState(), indexer.getState().next());
        telemetry.addData("Indexer Voltages",
                "Target: %.3f , Actual: %.3f",
                indexer.getTargetVoltage(), indexer.getVoltage());
        telemetry.addData("Outtake RPM", outtake.getRPM());
        telemetry.addData("Target RMP", outtake.getTargetRPM());
        telemetry.addData("Actuator up?", actuator.isActivated());
        telemetry.addData("Indexer Loaded?", indexer.isLoaded());
        telemetry.addData("April Lock", continuousAprilTagLock);
        telemetry.addData("Bot Range", aprilTag.getRange());
        telemetry.addData("Alliance selected", colorGoalSelected);
        telemetry.addData("Turn Correction:", turnCorrection);
        telemetry.addData("Intake power: ", intake.getIntakingPower());
        telemetry.addData("Last Turn Correction", lastTurnCorrection);
        for (Indexer.IndexerState s : Indexer.IndexerState.values()) {
            telemetry.addData(
                    "Slot " + s.index,
                    "%s (err=%.1f°)",
                    indexer.getColorAt(s),
                    indexer.debugSlotErrorDeg(s)
            );
        }
        telemetry.update();
    }

    protected void handleAllianceSelection() {
        if (g1.wasJustPressed(GamepadKeys.Button.BACK)) {
            aprilTag.setPipeline(0);
            colorGoalSelected = "Blue";
        }
        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            aprilTag.setPipeline(1);
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
        if (fieldCentric) movement.teleopTickFieldCentric(lx, ly, rx, turnCorrection, true);
        else movement.teleopTick(lx, ly, rx, turnCorrection);
    }

    protected void handleAprilTagLock() {
        // Toggle continuous lock with gamepad1 A
        if (g1.wasJustPressed(GamepadKeys.Button.A)) {
            continuousAprilTagLock = true;
            g1.gamepad.rumbleBlips(2);
        }
        if (g1.wasJustPressed(GamepadKeys.Button.B)) {
            continuousAprilTagLock = false;
            g1.gamepad.rumbleBlips(1);
        }

        if (continuousAprilTagLock) {
            long now = System.currentTimeMillis();
            if (now - lastAimUpdate >= AIM_UPDATE_INTERVAL_MS) {
                lastAimUpdate = now;
                aprilTag.scanGoalTag();
                double bearing = aprilTag.getBearing();

                if (!Double.isNaN(bearing)) {
                    lastTurnCorrection = aprilAimer.calculateTurnPowerFromBearing(bearing);
                    turnCorrection = lastTurnCorrection;
                } else {
                    turnCorrection = 0;
                }
            }

            if (!Double.isNaN(aprilTag.getRange())) {
                targetRPM = outtake.getRegressionRPM(aprilTag.getRange() + rangeOffset);
            }
            /*else {
                targetRPM = 3800;
            }*/
        } else {
            turnCorrection = 0;
            //targetRPM = 3800;
        }
    }

    protected double getTargetRPM() {
        return targetRPM;
    }
}

