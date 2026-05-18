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
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

@Config
public class Bot extends BotPeriodics {
    // haptics & lights
    private boolean rumbledAlready = false;
    public enum FSM {
        Intake,
        Outtake
    }

    public FSM state;
    public static double withinRpmRange = 150; //
    public static double FIRE_TIME = 3.0;

    public Bot(HardwareMap hardwareMap, Telemetry tele, MecanumDrive mecanumDrive, Gamepad gamepad1, Gamepad gamepad2, boolean twoMovement) {
        super(hardwareMap, tele, mecanumDrive, gamepad1, gamepad2, twoMovement);
        state = FSM.Outtake;
        hardwareMap.voltageSensor.iterator().next().getVoltage(); // ensure voltage sensor is initialized before teleop starts
    }

    public void teleopInit() {
        state = FSM.Outtake;
        outtake.stop();
    }

    public void teleopStart(){
        //TODO: Determine necessity of this john
    }

    public void teleopTick()
    {
        handlePeriodics();
        switch (state) {
            case Intake:
                handleIntakeState();
                break;
            case Outtake:
                handleOuttakeState();
                break;
        }
    }

    // MAINLINE HANDLERS
    private void handleIntakeState() {
        double leftTrigger = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        //double rightTrigger = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);
        // Press-and-hold right bumper to spin up shooter while in Intake
        //safety measure - until you purposefully spin up (or change states, it can't shoot)
        if (!actionHost.isRunning()) {
            if (g2.gamepad.right_bumper) {
                rangeRequested = true;
                state = FSM.Outtake;
                applyPreSpinRPM();
            } else {
                outtake.stop();
                rangeRequested = false;
            }
        }

        if (g2.wasJustPressed(GamepadKeys.Button.A))
            state = FSM.Outtake;

        if(storage.isFull() && !rumbledAlready && !g1.gamepad.isRumbling() && !g2.gamepad.isRumbling()){
            g1.gamepad.rumbleBlips(TeleopConstants.Gamepad.FULL_WARNING_RUMBLES);
            g2.gamepad.rumbleBlips(TeleopConstants.Gamepad.FULL_WARNING_RUMBLES);
            rumbledAlready = true;
        }
    }

    protected void handleAllianceSelection() {
        if (g1.wasJustPressed(GamepadKeys.Button.BACK)) {
            aimer.setBlueTarget();
            g1.gamepad.setLedColor(0, 0, 1, TeleopConstants.Gamepad.GAMEPAD_LIGHT_COLOR_DURATION);
            colorGoalSelected = "Blue";
        }
        if (g1.wasJustPressed(GamepadKeys.Button.START)) {
            aimer.setRedTarget();
            g1.gamepad.setLedColor(1, 0, 0, TeleopConstants.Gamepad.GAMEPAD_LIGHT_COLOR_DURATION);
            colorGoalSelected = "Red";
        }
    }

    private void handleOuttakeState() {
        // Allow press-and-hold pre-spin while in QuickOuttake (before running actions)
        if (!actionHost.isRunning()) {
            if (g2.gamepad.right_bumper) {
                applyPreSpinRPM();
                rangeRequested = true;
            } else {
                rangeRequested = false;
                outtake.stop();
            }
        }

        if (!actionHost.isRunning() && g2.wasJustPressed(GamepadKeys.Button.X)) {
            actionHost.start(actionFire());
            rumbledAlready = false;
            state = FSM.Intake;

    }}

    private void applyPreSpinRPM() {
        outtake.set(2000); // RPM mode: set shooter target RPM
    }

    private Action actionFire() {
        final double rpm = 2000;
        return new SequentialAction(
                new InstantAction(() -> outtake.set(rpm)),
                packet -> {
                    outtake.set(2000);
                    return !outtake.inRange(withinRpmRange);
                },
                new InstantAction(storage::openGate),
                new SleepAction(FIRE_TIME),
                new InstantAction(storage::closeGate),
                new InstantAction(outtake::stop)
        );
    }
}
