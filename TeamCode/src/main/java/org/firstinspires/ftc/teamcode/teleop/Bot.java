package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@Config
public class Bot extends BotPeriodics {
    // haptics & lights
    private boolean rumbledAlready = false;
    private int fullWarningRumbles = 3;
    private int gamepadLightColorDuration = 500;
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

    public Bot(HardwareMap hardwareMap, Telemetry tele, Gamepad gamepad1, Gamepad gamepad2) {
        super(hardwareMap, tele, gamepad1, gamepad2);
        state = FSM.Intake;
    }

    public void teleopInit() {
        actuator.down();
        indexer.initializeColors(Indexer.ArtifactColor.EMPTY);
        indexer.moveTo(Indexer.IndexerState.zero);
        indexer.setIntaking(true);
        state = FSM.Intake;
        actionHost = new BotPeriodics.ActionHost();
    }

    public void teleopTick()
    {
        handlePeriodics();
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
    }
    // MAINLINE HANDLERS
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

    protected void handleAllianceSelection() {
        if (g1.wasJustPressed(GamepadKeys.Button.BACK)) {
            goalTagID = 20;
            aprilTag.setGoalTagID(goalTagID);
            g1.gamepad.setLedColor(0, 0, 1, gamepadLightColorDuration);
            colorGoalSelected = "Blue";
        }
        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            goalTagID = 24;
            aprilTag.setGoalTagID(goalTagID);
            g1.gamepad.setLedColor(1, 0, 0, gamepadLightColorDuration);
            colorGoalSelected = "Red";
        }
    }

    private void handleQuickOuttakeState() {
        if (!actionHost.isRunning() && g2.wasJustPressed(GamepadKeys.Button.X)) {
            actionHost.start(actionNonIndexedDump());
            rumbledAlready = false;
        }
        if (g2.wasJustPressed(GamepadKeys.Button.BACK)) {
            actionHost.abort();
        }
        if (g2.wasJustPressed(GamepadKeys.Button.A)) {
            state = FSM.Intake;
            indexer.setIntaking(true);
        }
    }

    private void handleSortOuttakeState() {
        if (!actionHost.isRunning()) {
            if (g2.wasJustPressed(GamepadKeys.Button.X)) {
                actionHost.start(actionFireGreen());
                rumbledAlready = false;
            }
            if (g2.wasJustPressed(GamepadKeys.Button.Y)) {
                actionHost.start(actionFirePurple());
                rumbledAlready = false;
            }
        }
        if (g2.wasJustPressed(GamepadKeys.Button.BACK)) {
            actionHost.abort();
        }
        if (g2.wasJustPressed(GamepadKeys.Button.A)) {
            indexer.setIntaking(true);
            state = FSM.Intake;
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
                new InstantAction(indexer::initializeColors),
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

    private double getTargetRpm() {
        double range = aprilTag.getRange();
        if (Double.isNaN(range) || range <= 0) {
            return shooterRPM;
        }
        return outtake.getRegressionRPM(range);
    }
}