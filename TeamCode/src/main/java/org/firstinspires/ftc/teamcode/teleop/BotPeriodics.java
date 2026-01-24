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
    protected final IMU imu;
    protected final TwoDeadWheelLocalizer deadWheelLocalizer;

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

    public static double targetRPM = 0;
    protected static final long AIM_UPDATE_INTERVAL_MS = 50;

    public BotPeriodics(HardwareMap hardwareMap, Telemetry tele, Gamepad gamepad1, Gamepad gamepad2) {
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        actuator = new Actuator(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        ConcreteLazyImu concreteImu = new ConcreteLazyImu(hardwareMap, "imu", new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));
        movement = new Movement(hardwareMap, concreteImu);
        imu = movement.getImu();
        deadWheelLocalizer = movement.getTwoDeadWheelLocalizer();
        aprilTag = new AprilTag(hardwareMap, tele);
        aprilAimer = new AprilTagAimer(hardwareMap, imu, deadWheelLocalizer);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        actionHost = new ActionHost();
        telemetry = tele;
    }
    
    protected void handlePeriodics()
    {
        g1.readButtons();
        g2.readButtons();
        // driver one (constant
        handleAprilTagLock();
        handleMovement();
        handleAllianceSelection();

        handleTelemetry();

        indexer.update();
        outtake.periodic();
        actionHost.update();
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
            goalTagID = 20;
            aprilTag.setGoalTagID(goalTagID);
            colorGoalSelected = "Blue";
        }
        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            goalTagID = 24;
            aprilTag.setGoalTagID(goalTagID);
            colorGoalSelected = "Red";
        }
    }

    protected void handleMovement() {
        double lx = g1.getLeftX();
        double ly = g1.getLeftY();
        double rx = g1.getRightX();

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
                targetRPM = outtake.getRegressionRPM(aprilTag.getRange());
            }
        } else {
            turnCorrection = 0;
        }
    }

    protected double getTargetRPM() {
        return targetRPM;
    }
}

