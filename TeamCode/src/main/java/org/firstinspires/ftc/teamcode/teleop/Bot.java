package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
    public enum FSM {
        MotifSelection,
        Intake,
        QuickOuttake,
        SortOuttake,
        Endgame
    }

    public FSM state;

    public static double NON_INDEX_SPIN_TIME = 3; //seconds of full-power indexer blast
    public static double SHOOTER_SPINUP = 2.0;
    public static double FULL_BLAST_POWER = 0.25;
    public static double QUICKSPIN_OUTTAKE_RPM_SCALE = 1.12;

    public Indexer.ArtifactColor[] motif;

    private Indexer.ArtifactColor[] PPG = new Indexer.ArtifactColor[]{Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.GREEN};
    private Indexer.ArtifactColor[] PGP = new Indexer.ArtifactColor[]{Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE};
    private Indexer.ArtifactColor[] GPP = new Indexer.ArtifactColor[]{Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.PURPLE};

    // Press-and-hold pre-spin minimum RPM
    public static double INTAKE_MIN_RPM = 3500.0;

    public Bot(HardwareMap hardwareMap, Telemetry tele, Gamepad gamepad1, Gamepad gamepad2) {
        super(hardwareMap, tele, gamepad1, gamepad2);
        state = FSM.MotifSelection;
    }

    public void teleopInit() {
        indexer.initializeColors(Indexer.ArtifactColor.EMPTY);
        indexer.setIntaking(true);
        state = FSM.MotifSelection;
        outtake.stop();
    }

    public void teleopStart(){
        actuator.down();
        indexer.moveTo(Indexer.IndexerState.zero);
    }

    public void teleopTick()
    {
        handlePeriodics();
        switch (state) {
            case MotifSelection:
                if (g2.wasJustPressed(GamepadKeys.Button.X)){
                    motif = PPG;
                    indexer.prepareQuickspin(motif);
                    state = FSM.Intake;
                }
                if (g2.wasJustPressed(GamepadKeys.Button.Y)){
                    motif = PGP;
                    indexer.prepareQuickspin(motif);
                    state = FSM.Intake;
                }
                if (g2.wasJustPressed(GamepadKeys.Button.B)){
                    motif = GPP;
                    indexer.prepareQuickspin(motif);
                    state = FSM.Intake;
                }
                break;
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

        // Press-and-hold right bumper to spin up shooter while in Intake
        if (!actionHost.isRunning()) {
            if (g2.gamepad.right_bumper) {
                state = FSM.QuickOuttake;
                applyPreSpinRPM();
            } else {
                outtake.stop();
            }
        }


        if (leftTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE) intake.run();
        else intake.stop();

        //if (rightTrigger > TRIGGER_DEADZONE) intake.runBackwards();
        //else intake.stop();

        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) indexer.moveTo(indexer.getState().next());

        if (g2.wasJustPressed(GamepadKeys.Button.A))
            state = FSM.QuickOuttake;
        if (g2.wasJustPressed(GamepadKeys.Button.B)){
            state = FSM.SortOuttake;
            indexer.setIntaking(false);
            indexer.moveTo(indexer.getState());
        }
        if(g2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            indexer.prepareQuickspin(new Indexer.ArtifactColor[]{Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE,Indexer.ArtifactColor.PURPLE});
        if (g2.wasJustPressed(GamepadKeys.Button.Y)) state = FSM.Endgame;

        if(indexer.isLoaded() && !rumbledAlready && !g1.gamepad.isRumbling() && !g2.gamepad.isRumbling()){
            g1.gamepad.rumbleBlips(TeleopConstants.Gamepad.FULL_WARNING_RUMBLES);
            g2.gamepad.rumbleBlips(TeleopConstants.Gamepad.FULL_WARNING_RUMBLES);
            rumbledAlready = true;
        }
    }

    protected void handleAllianceSelection() {
        if (g1.wasJustPressed(GamepadKeys.Button.BACK)) {
            goalTagID = 20;
            aprilTag.setGoalTagID(goalTagID);
            g1.gamepad.setLedColor(0, 0, 1, TeleopConstants.Gamepad.GAMEPAD_LIGHT_COLOR_DURATION);
            colorGoalSelected = "Blue";
        }
        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            goalTagID = 24;
            aprilTag.setGoalTagID(goalTagID);
            g1.gamepad.setLedColor(1, 0, 0, TeleopConstants.Gamepad.GAMEPAD_LIGHT_COLOR_DURATION);
            colorGoalSelected = "Red";
        }
    }

    private void handleQuickOuttakeState() {
        // Allow press-and-hold pre-spin while in QuickOuttake (before running actions)
        if (!actionHost.isRunning()) {
            if (g2.gamepad.right_bumper) {
                applyPreSpinRPM();
            } else {
                outtake.stop();
            }
        }

        if (!actionHost.isRunning() && g2.wasJustPressed(GamepadKeys.Button.X)) {
            actionHost.start(actionNonIndexedDump());
            rumbledAlready = false;
            state = FSM.Intake;
        }
        if(g2.wasJustPressed(GamepadKeys.Button.DPAD_UP))
            indexer.prepareQuickspin(new Indexer.ArtifactColor[]{Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE,Indexer.ArtifactColor.PURPLE});
        if (g2.wasJustPressed(GamepadKeys.Button.BACK)) {
            actionHost.abort();
        }
        if (g2.wasJustPressed(GamepadKeys.Button.A)) {
            state = FSM.Intake;
            indexer.setIntaking(true);
            rumbledAlready = false;
        }
    }

    private void handleSortOuttakeState() {
        if (!actionHost.isRunning()) {
            if (g2.wasJustPressed(GamepadKeys.Button.X)) {
                actionHost.start(actionFireGreen());
            }
            if (g2.wasJustPressed(GamepadKeys.Button.Y)) {
                actionHost.start(actionFirePurple());
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

    private void applyPreSpinRPM() {
        outtake.set(getTargetRPM()); // RPM mode: set shooter target RPM
    }

    private Action actionNonIndexedDump() {
        final double rpm = getTargetRPM() * QUICKSPIN_OUTTAKE_RPM_SCALE;
        return new SequentialAction(
                new InstantAction(actuator::upQuick),
                new InstantAction(() -> outtake.set(rpm)),
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket packet) {
                        return !outtake.inRange(100.0);
                    }
                },
                new InstantAction(() -> indexer.setIndexerPower(FULL_BLAST_POWER)),
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

        final double rpm = getTargetRPM();

        return new SequentialAction(
                new InstantAction(() -> indexer.setIntaking(false)),
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.moveTo(slot, true)),
                new InstantAction(() -> outtake.set(rpm)),
                // wait for shooter RPM to be within 100 instead of fixed spinup
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket p) {
                        return !outtake.inRange(100.0);
                    }
                },
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
            return new InstantAction(() -> {
            });
        }

        final double rpm = getTargetRPM();

        return new SequentialAction(
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.moveTo(slot, true)),

                new InstantAction(() -> outtake.set(rpm)),
                // wait for shooter RPM to be within 100 instead of fixed spinup
                new Action() {
                    @Override
                    public boolean run(TelemetryPacket p) {
                        return !outtake.inRange(100.0);
                    }
                },
                new InstantAction(actuator::upIndexed),
                new SleepAction(1),

                new InstantAction(() ->
                        indexer.assignSlotColor(slot, Indexer.ArtifactColor.EMPTY)
                ),

                new InstantAction(outtake::stop),
                new InstantAction(actuator::down)
        );
    }
}
