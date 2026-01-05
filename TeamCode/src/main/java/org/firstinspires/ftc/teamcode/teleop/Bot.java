package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Movement;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Config
public class Bot {
    private final Intake intake;
    private final Indexer indexer;
    private final Actuator actuator;
    private final Outtake outtake;
    private final Movement movement;
    private final AprilTag aprilTag;
    private final AprilTagAimer aprilAimer;
    private final IMU imu;
    private final TwoDeadWheelLocalizer deadWheelLocalizer;

    private final GamepadEx g1;
    private final GamepadEx g2;
    private final Telemetry telemetry;

    // haptics & lights
    private boolean rumbledAlready = false;
    private int fullWarningRumbles = 3;
    private int gamepadLightColorDuration = 500;


    // camera vision
    private boolean fieldCentric = false;
    private boolean continuousAprilTagLock = false;
    private long lastAimUpdate = 0;
    private double lastTurnCorrection = 0.0;
    private double turnCorrection = 0.0;
    private int goalTagID;
    private String colorGoalSelected;
    public enum FSM {
        Intake,
        QuickOuttake,
        SortOuttake,
        Endgame
    }

    public FSM state;

    public static double TRIGGER_DEADZONE = 0.05;
    public static double shooterRPM = 2900;
    public static double NON_INDEX_SPIN_TIME = 3;//seconds of full-power indexer blast
    public static double SHOOTER_SPINUP = 2.0;
    public static double FULL_BLAST_POWER =0.25;
    public static double QUICKSPIN_OUTTAKE_RPM_SCALE = 1.12;
    private static final long AIM_UPDATE_INTERVAL_MS = 50;

    public Bot(HardwareMap hardwareMap, Telemetry tele, Gamepad gamepad1, Gamepad gamepad2) {
        intake = new Intake(hardwareMap);
        indexer = new Indexer(hardwareMap);
        actuator = new Actuator(hardwareMap);
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        movement = new Movement(hardwareMap);
        imu = movement.getImu();
        deadWheelLocalizer = movement.getTwoDeadWheelLocalizer();
        aprilTag = new AprilTag(hardwareMap, tele);
        aprilAimer = new AprilTagAimer(hardwareMap, imu, deadWheelLocalizer);
        g1 = new GamepadEx(gamepad1);
        g2 = new GamepadEx(gamepad2);
        telemetry = tele;
        state = FSM.Intake;
    }

    public void teleopInit() {
        actuator.down();
        indexer.initializeColors(Indexer.ArtifactColor.EMPTY);
        indexer.moveTo(Indexer.IndexerState.zero);
        indexer.setIntaking(true);
        state = FSM.Intake;
    }

    public void teleopTick() {
        g1.readButtons();
        g2.readButtons();

        handleAprilTagLock();
        handleMovement();

        if (g1.wasJustPressed(GamepadKeys.Button.BACK)) {
            goalTagID = 20;
            aprilTag.setGoalTagID(goalTagID); // blue
            g1.gamepad.setLedColor(0,0,1, gamepadLightColorDuration);
            colorGoalSelected = "Blue";
        }

        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            goalTagID = 24;
            aprilTag.setGoalTagID(goalTagID); // red
            g1.gamepad.setLedColor(1,0,0, gamepadLightColorDuration);
            colorGoalSelected = "Red";
        }

        outtake.periodic();
        indexer.update();

        switch (state) {
            case Intake:
                handleIntakeState();
                break;
            case QuickOuttake:
                handleQuickOuttakeState();
                break;
            case SortOuttake:
                handleSortOuttakeState();
                break;
            case Endgame:
                handleEndgameState();
                break;
        }

        telemetry.addData("Field Centric", fieldCentric);
        telemetry.addData("Indexer State", "%s -> %s", indexer.getState(), indexer.getState().next());
        telemetry.addData("Indexer Voltages", "Target: %.3f , Actual: %.3f", indexer.getTargetVoltage(), indexer.getVoltage());
        telemetry.addData("Outtake RPM", "Target: %.1f, Actual: %.1f", outtake.getTargetRPM(), outtake.getRPM());
        telemetry.addData("Actuator up?", actuator.isActivated());
        telemetry.addData("Indexer Loaded?", indexer.isLoaded());
        telemetry.addData("April Lock", continuousAprilTagLock);
        telemetry.addData("Bot Range", aprilTag.getRange());
        telemetry.addData("Alliance selected:", colorGoalSelected);
        for (Indexer.IndexerState s : Indexer.IndexerState.values()) {
            telemetry.addData(
                    "Slot " + s.index,
                    "%s  (err=%.1f°)",
                    indexer.getColorAt(s),
                    indexer.debugSlotErrorDeg(s)
            );
        }
        telemetry.update();
    }

    private void handleMovement() {
        double lx = g1.getLeftX();
        double ly = g1.getLeftY();
        double rx = g1.getRightX();

        if (fieldCentric) movement.teleopTickFieldCentric(lx, ly, rx, turnCorrection, true);
        else movement.teleopTick(lx, ly, rx, turnCorrection);
    }

    private void handleIntakeState() {
        double leftTrigger = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        //double rightTrigger = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (leftTrigger > TRIGGER_DEADZONE) intake.run();
        else intake.stop();

        //if (rightTrigger > TRIGGER_DEADZONE) intake.runBackwards();
        //else intake.stop();

        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) indexer.moveTo(indexer.getState().next());

        if (g2.wasJustPressed(GamepadKeys.Button.A)) state = FSM.QuickOuttake;
        if (g2.wasJustPressed(GamepadKeys.Button.B)){
            state = FSM.SortOuttake;
            indexer.setIntaking(false);
            indexer.moveTo(indexer.getState());
        }
        if (g2.wasJustPressed(GamepadKeys.Button.Y)) state = FSM.Endgame;

        if(indexer.isLoaded() && !rumbledAlready && !g1.gamepad.isRumbling() && !g2.gamepad.isRumbling()){ // works with my other code in the outtake functions to ensure warning rumbles don't happen more than once
            g1.gamepad.rumbleBlips(fullWarningRumbles);
            g2.gamepad.rumbleBlips(fullWarningRumbles);
            rumbledAlready = true;
        }
    }

    private void handleQuickOuttakeState() {
        if (g2.wasJustPressed(GamepadKeys.Button.X)) {
            Actions.runBlocking(fireWithPeriodic(actionNonIndexedDump()));
            rumbledAlready = false;
        }
        if (g2.wasJustPressed(GamepadKeys.Button.A)) {
            state = FSM.Intake;
            indexer.setIntaking(true);
            rumbledAlready = false;
        }
    }

    private void handleSortOuttakeState() {
        if (g2.wasJustPressed(GamepadKeys.Button.X)) {
            Actions.runBlocking(fireWithPeriodic(actionFireGreen()));
            rumbledAlready = false;
        }
        if (g2.wasJustPressed(GamepadKeys.Button.Y)) {
            Actions.runBlocking(fireWithPeriodic(actionFirePurple()));
            rumbledAlready = false;
        }
        if (g2.wasJustPressed(GamepadKeys.Button.A)) {
            indexer.setIntaking(true);
            state = FSM.Intake;
            rumbledAlready = false;
        }
    }

    private void handleEndgameState() {
        if (g2.wasJustPressed(GamepadKeys.Button.A)) {
            state = FSM.Intake;

            indexer.setIntaking(true);
        }
    }

    private Action actionNonIndexedDump() {
        final double rpm = getTargetRpm() * QUICKSPIN_OUTTAKE_RPM_SCALE;
        return new SequentialAction(
                new InstantAction(actuator::upQuick),// lower up position for quick dump
                new InstantAction(() -> outtake.set(rpm)),
                new SleepAction(SHOOTER_SPINUP),                      // spin up shooter
                new InstantAction(() -> indexer.setIndexerPower(FULL_BLAST_POWER)),// full blast
                new SleepAction(NON_INDEX_SPIN_TIME),
                new InstantAction(indexer::stopIndexerPower),
                new InstantAction(outtake::stop),
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.setIntaking(true)),
                new InstantAction(() -> indexer.initializeColors()),
                new InstantAction(() -> indexer.moveTo(Indexer.IndexerState.zero))
        );
    }

    private Action actionFireGreen() {
        final Indexer.IndexerState slot =
                indexer.findBestSlotForColor(Indexer.ArtifactColor.GREEN);

        if (slot == null) {
            return new InstantAction(() -> {});
        }

        final double rpm = getTargetRpm();

        return new SequentialAction(
                new InstantAction(() -> indexer.setIntaking(false)),
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.moveTo(slot, true)),
                new InstantAction(() -> outtake.set(rpm)),
                new SleepAction(SHOOTER_SPINUP),
                new InstantAction(actuator::upIndexed),
                new SleepAction(1),

                new InstantAction(() ->
                        indexer.assignSlotColor(slot, Indexer.ArtifactColor.EMPTY)
                ),

                new InstantAction(outtake::stop),
                new InstantAction(actuator::down)
        );
    }

    private Action actionFirePurple() {
        final Indexer.IndexerState slot =
                indexer.findBestSlotForColor(Indexer.ArtifactColor.PURPLE);

        if (slot == null) {
            return new InstantAction(() -> {});
        }

        final double rpm = getTargetRpm();

        return new SequentialAction(
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.moveTo(slot, true)),

                new InstantAction(() -> outtake.set(rpm)),
                new SleepAction(SHOOTER_SPINUP),
                new InstantAction(actuator::upIndexed),
                new SleepAction(1),

                new InstantAction(() ->
                        indexer.assignSlotColor(slot, Indexer.ArtifactColor.EMPTY)
                ),

                new InstantAction(outtake::stop),
                new InstantAction(actuator::down)
        );
    }



    private Action fireWithPeriodic(Action fireAction) {
        return packet -> {
            indexer.update();
            outtake.periodic();
            handleMovement();
            handleAprilTagLock();
            g1.readButtons();
            return fireAction.run(packet);
        };
    }

    private void handleAprilTagLock() {
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
            lastTurnCorrection = 0;
            turnCorrection = 0;
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

            if (lastTurnCorrection != 0 && !Double.isNaN(lastTurnCorrection)) {
                shooterRPM = (int) outtake.getRegressionRPM(aprilTag.getRange());
            }
            turnCorrection = 0.9 * lastTurnCorrection;
        }
    }

    private double getTargetRpm() {
        double range = aprilTag.getRange();
        if (Double.isNaN(range) || range <= 0) {
            return shooterRPM;
        }
        return outtake.getRegressionRPM(range);
    }
}
