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
    public static double BEARING_AVOID_IN_DEGREES = 26.7;
    protected ActionHost actionHost;
    // camera vision

    public static boolean fieldCentric = false;
    public static boolean drivetrainAim = false;
    protected boolean aimlock = false;
    public static boolean useMecanumAvoid = true;
    protected boolean mecanumAvoiding = false;
    private double servoOffset;
    protected long lastAimUpdate = 0;
    protected double lastTurnCorrection = 0.0;
    protected double turnCorrection = 0.0;
    protected String colorGoalSelected = "";
    protected double[] targetData = {0,0,0};

    public static boolean continuousIntake = true;
    public static boolean manualTurretAim = false;
    private double manualTurretTarget = 0;
    public static double turretlLeftStickMult = 5.0;
    public static double turretRightStickMult = 2.0;

    public static double targetRPM = 2000;
    public static double outtakeEjectRpm = 670;
    protected static final long AIM_UPDATE_INTERVAL_MS = 0;
    protected boolean initialBackwardsTransfer = true;

    protected boolean twoMovementMode;
    protected boolean isFarShooting = false;

    protected boolean rumbledAlready = false;
    protected boolean currentAutoAimlock = true;
    protected boolean turnCurrentSensingOff = false;

    public BotPeriodics(HardwareMap hardwareMap, Telemetry tele, MecanumDrive mecanumDrive, Gamepad gamepad1, Gamepad gamepad2, boolean useMovement, boolean isFarShooting) {
        this.isFarShooting = isFarShooting;
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        drive = mecanumDrive;
        storage = new Storage(hardwareMap, intake);
        turret = new Turret(hardwareMap);
        movement = new Movement(hardwareMap, drive);
        aimer = new Aimer(drive, isFarShooting);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        actionHost = new ActionHost();
        telemetry = tele;
        twoMovementMode = useMovement;
        servoOffset = turret.getServoOffset();
    }

    protected void handlePeriodics()
    {
        g1.readButtons();
        g2.readButtons();

        if (!actionHost.isRunning()) {
            handleIntake();
            handleOuttake();
            handleStorage();
            handleAllianceSelection();
        }

        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_LEFT) && aimlock == false) {
            manualTurretAim = true;
        }
        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
            manualTurretAim = false;
        }

        if (g2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            turnCurrentSensingOff = false;
        }
        else if (g2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            turnCurrentSensingOff = true;
        }

        if (manualTurretAim) {
            handleManualTurret();
        }

        handleAimLock();
        handleMovement();
        handleTelemetry();

        if (!initialBackwardsTransfer && currentAutoAimlock)
            storage.updateForIfFull();

        outtake.periodic();
        actionHost.update();
        drive.updatePoseEstimate();

        long now = System.currentTimeMillis();

        if (now - lastAimUpdate >= AIM_UPDATE_INTERVAL_MS) {
            lastAimUpdate = now;
            targetData = aimer.calculateLocalizedData();
            lastTurnCorrection = targetData[0];
            targetRPM = outtake.getRegressionRPM(targetData[1], isFarShooting);
            bearingTurnCorrection = targetData[2];
        }
        turnCorrection = lastTurnCorrection;
    }
    private void handleIntake() {
        double leftTrigger = g1.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        double rightTrigger = g1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (rightTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE) {
            intake.run();
            if (!initialBackwardsTransfer)
                storage.runTransfer(isFarShooting);
        }
        else if (leftTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE) {
            intake.runBackwards(); // only run intake backwards to eject only 4th ball
            if (!initialBackwardsTransfer)
                storage.stopTransfer();
        }
        else if (continuousIntake) {
            intake.runSlow();
            if (!initialBackwardsTransfer)
                storage.stopTransfer();
        }
        else {
            intake.stop();
            if (!initialBackwardsTransfer)
                storage.stopTransfer();
        }

        if (leftTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE) {
            setCurrentAutoAimlock(false);
            setRumbledAlready(false);
        }
        else {
            setCurrentAutoAimlock(true);
        }
    }

    private void handleOuttake() {
        double rightTrigger = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (rightTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE)
            outtake.set(getTargetRPM());
        else if (g2.getButton(GamepadKeys.Button.B))
            outtake.set(outtakeEjectRpm);
        else outtake.stop();
    }

    private void handleStorage() {
        GamepadKeys.Button openGateButton = GamepadKeys.Button.DPAD_UP;
        GamepadKeys.Button closeGateButton = GamepadKeys.Button.DPAD_DOWN;
        double leftTrigger = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);

        if (g2.wasJustPressed(openGateButton)) {
            storage.openGate();
        } else if (g2.wasJustPressed(closeGateButton)) {
            storage.closeGate();
        }

        if (initialBackwardsTransfer || g2.getButton(GamepadKeys.Button.X)) {
            storage.runTransferBackwards();
        } else if (leftTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE) {
            if (!initialBackwardsTransfer)
                storage.runTransfer(isFarShooting);
            intake.run();
        } else if (g1.getButton(GamepadKeys.Button.Y) || g2.getButton(GamepadKeys.Button.Y)) {
            storage.runTransferBackwards();
            intake.runBackwards();
        }

        if (g1.getButton(GamepadKeys.Button.Y) || g2.getButton(GamepadKeys.Button.Y)) {
            setCurrentAutoAimlock(false);
            setRumbledAlready(false);
        } else {
            setCurrentAutoAimlock(true);
        }
    }

    private void handleManualTurret() {
        double lx = g2.getLeftX();
        double rx = g2.getRightX();

        if (Math.abs(lx) >= 0.1) {
            manualTurretTarget = wrapAngle360(manualTurretTarget + lx * turretlLeftStickMult);
        }
        if (Math.abs(rx) >= 0.01) {
            manualTurretTarget = wrapAngle360(manualTurretTarget + rx * turretRightStickMult);
        }

        double servoPosition = manualTurretTarget / turret.getActualRangeOfMotion(); // deadzone is representative of the actual angle
        turret.setServos(servoPosition);
    }

    // Periodic Handlers
    protected void handleTelemetry()
    {
        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Outtake RPM", outtake.getMeasuredRPM());
        telemetry.addData("Target RMP", outtake.getTargetRPM());
        telemetry.addData("Bot Range", targetData[1]);
        telemetry.addData("x", drive.localizer.getPose().position.x);
        telemetry.addData("y", drive.localizer.getPose().position.y);
        telemetry.addData("Alliance selected", colorGoalSelected);
        telemetry.addData("Turn Correction:", turnCorrection);
        telemetry.addData("Manual Turret Angle", manualTurretTarget);
        telemetry.addData("Intake power: ", intake.getPower());
        telemetry.addData("Last Turn Correction", lastTurnCorrection);
        telemetry.addData("Last Turn Correction", lastTurnCorrection);
        telemetry.addData("Transfer and intake Current (amps): ", storage.getCurrent());
        telemetry.update();
    }

    protected void handleAllianceSelection() {
        if (g1.wasJustPressed(GamepadKeys.Button.BACK) || g2.wasJustPressed(GamepadKeys.Button.BACK)) {
            aimer.setBlueTarget();
            colorGoalSelected = "Blue";
        }
        if (g1.wasJustPressed(GamepadKeys.Button.START) || g2.wasJustPressed(GamepadKeys.Button.START)) {
            aimer.setRedTarget();
            colorGoalSelected = "Red";
        }
    }

    protected void handleMovement() {
        double lx = g1.getLeftX();
        double ly = g1.getLeftY();
        double rx = g1.getRightX();

        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON) || g2.wasJustPressed(GamepadKeys.Button.LEFT_STICK_BUTTON)) {
            movement.setSlow(true);
        }
        else if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON) || g2.wasJustPressed(GamepadKeys.Button.RIGHT_STICK_BUTTON)) {
            movement.setSlow(false);
        }

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
                            lx,
                            ly,
                            rx,
                            turnCorrection
                    );
                }
            } else {
                if (useMecanumAvoid && mecanumAvoiding) {
                    // Avoiding the heading where servo must wraparound, to disable set BEARING_AVOID_IN_DEGREES = 0
                    double targetAngle = wrapAngle360(-bearingTurnCorrection + 180 + turret.getServoOffset());

                    // The center of the deadzone
                    double deadZoneHalfway = turret.getActualRangeOfMotion() + ((360.0 - turret.getActualRangeOfMotion()) / 2.0);
                    if (targetAngle > turret.getActualRangeOfMotion()) {
                        if (targetAngle < deadZoneHalfway)
                            targetAngle = turret.getActualRangeOfMotion();
                        else
                            targetAngle = 0.0;
                    }

                    double currentServoTargetPos = targetAngle / turret.getActualRangeOfMotion();

                    // 0 to 1, or 0 to servo physical range of motion
                    double lowerLimit = BEARING_AVOID_IN_DEGREES / turret.getActualRangeOfMotion();
                    double upperLimit = 1.0 - BEARING_AVOID_IN_DEGREES / turret.getActualRangeOfMotion();

                    if (currentServoTargetPos >= upperLimit) {
                        // Move counterclockwise for degrees past limit
                        double degreesPastLimit = (currentServoTargetPos - upperLimit) * 360;
                        bearingAvoidCorrection = aimer.calculateTurnPowerFromBearing(degreesPastLimit);
                    } else if (currentServoTargetPos <= lowerLimit) {
                        // Move clockwise for degrees past limit
                        double degreesPastLimit = (lowerLimit - currentServoTargetPos) * 360;
                        bearingAvoidCorrection = aimer.calculateTurnPowerFromBearing(-degreesPastLimit);
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

                if (!manualTurretAim)
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
        if (g1.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER) || g2.wasJustPressed(GamepadKeys.Button.LEFT_BUMPER)) {
            aimlock = true;
            manualTurretAim = false;
            g1.gamepad.rumbleBlips(2);
            g2.gamepad.rumbleBlips(2);
        }
        if (g1.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER) || g2.wasJustPressed(GamepadKeys.Button.RIGHT_BUMPER)) {
            aimlock = false;
            setMecanumAvoiding(false);
            g1.gamepad.rumbleBlips(1);
            g2.gamepad.rumbleBlips(1);
        }
        if (g1.wasJustPressed(GamepadKeys.Button.DPAD_DOWN)) {
            aimer.relocalize();
        } else if (g1.wasJustPressed(GamepadKeys.Button.DPAD_UP)){
            aimer.localizeForFront();
        }

        if (targetData[1] <= 102 && aimlock) {
            setMecanumAvoiding(true);
        }
    }

    protected double getTargetRPM() {
        return targetRPM;
    }

    // wrapped to (-180, 180)
    protected double wrapAngleNegPos180(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }
    // wrapped to (0, 360)
    protected double wrapAngle360(double angle) {
        return ((angle%360)+360)%360;
    }

    protected void setAimlock(boolean lock) {
        aimlock = lock;
    }

    protected void setRumbledAlready(boolean rumbled) {
        rumbledAlready = rumbled;
    }
    protected void setCurrentAutoAimlock(boolean autoAimlock) {
        this.currentAutoAimlock = autoAimlock;
    }

    protected void setMecanumAvoiding(boolean mecanumAvoiding) {
        this.mecanumAvoiding = mecanumAvoiding;
    }
}

