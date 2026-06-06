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
    protected boolean aimlock = false;
    public static boolean mecanumAvoid = true;
    public static double servoOffset = 92.5;
    protected long lastAimUpdate = 0;
    protected double lastTurnCorrection = 0.0;
    protected double turnCorrection = 0.0;
    protected String colorGoalSelected = "";
    protected double[] targetData = {0,0,0};

    private boolean isFireActionRunning = false;
    private boolean manualTransfer = false;

    public static boolean continuousIntake = true;

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

        if (!actionHost.isRunning()) {
            handleIntake();
            handleOuttake();
            handleStorage();
        }

        handleAimLock();
        handleMovement();
        handleAllianceSelection();
        handleTelemetry();

        handleManualTurret();

        storage.updateForIfFull();
        outtake.periodic();
        actionHost.update();
        drive.updatePoseEstimate();

        long now = System.currentTimeMillis();

        if (now - lastAimUpdate >= AIM_UPDATE_INTERVAL_MS) {
            lastAimUpdate = now;
            targetData = aimer.calculateLocalizedData();
            lastTurnCorrection = targetData[0];
            targetRPM = outtake.getRegressionRPM(targetData[1]);
            bearingTurnCorrection = targetData[2];
        }
        turnCorrection = lastTurnCorrection;
    }
    private void handleIntake() {
        double leftTrigger = g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (rightTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE) {
            intake.run();
            if (!manualTransfer)
                storage.runTransfer();
        }
        else if (leftTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE) {
            intake.runBackwards(); // only run intake backwards to eject only 4th ball
            if (!manualTransfer)
                storage.stopTransfer();
        }
        else if (continuousIntake) {
            intake.runSlow();
            if (!manualTransfer)
                storage.stopTransfer();
        }
        else {
            intake.stop();
            if (!manualTransfer)
                storage.stopTransfer();
        }
    }

    private void handleOuttake() {
        double rightTrigger = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (rightTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE)
            outtake.set(getTargetRPM());
        else outtake.stop();
    }

    private void handleStorage() {
        GamepadKeys.Button openGateButton = GamepadKeys.Button.DPAD_UP;
        GamepadKeys.Button closeGateButton = GamepadKeys.Button.DPAD_DOWN;
        double leftTrigger = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        if (g2.wasJustPressed(openGateButton)) {
            storage.openGate();
        }
        else if (g2.wasJustPressed(closeGateButton)) {
            storage.closeGate();
        }

        if (leftTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE) {
            manualTransfer = true;
            storage.runTransfer();
        }
        else {
            manualTransfer = false;
            storage.stopTransfer();
        }
    }

    private void handleManualTurret() {
        // TODO:

        // get it to have stick 360 corresponding to turret360
        g2.getLeftX();
        g2.getLeftY();

        // for fine grained control
        g2.getRightX();

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
    }

    protected void handleAimLock() {
        // Toggle continuous lock
        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) || g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            aimlock = true;
            g1.gamepad.rumbleBlips(2);
            g2.gamepad.rumbleBlips(2);
        }
        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) || g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
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
        if (!aimlock){
            turnCorrection = 0;
        }
    }

    protected double getTargetRPM() {
        return targetRPM;
    }

    protected double angleWrapDegrees(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }

    protected void setAimlock(boolean lock) {
        aimlock = lock;
    }
}

