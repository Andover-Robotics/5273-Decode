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

    public static double withinRpmRange = 150; //
    public static double FIRE_TIME = 3.0;

    public Bot(HardwareMap hardwareMap, Telemetry tele, MecanumDrive mecanumDrive, Gamepad gamepad1, Gamepad gamepad2, boolean twoMovement) {
        super(hardwareMap, tele, mecanumDrive, gamepad1, gamepad2, twoMovement);
        hardwareMap.voltageSensor.iterator().next().getVoltage(); // ensure voltage sensor is initialized before teleop starts
    }

    public void teleopInit() {
        outtake.stop();
    }

    public void teleopStart(){
        //TODO: Determine necessity of this john
    }

    public void teleopTick()
    {
        handlePeriodics();
        handleIntakeFeedback();
        handleOuttakeActions();
    }

    // MAINLINE HANDLERS
    private void handleIntakeFeedback() {
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

    private void handleOuttakeActions() {
        if (!actionHost.isRunning() && g2.wasJustPressed(GamepadKeys.Button.A) && aimlock) {
            actionHost.start(actionFire());
            rumbledAlready = false;
        }
    }

    private Action actionFire() {
        final double rpm = getTargetRPM();
        return new SequentialAction(
                new InstantAction(() -> outtake.set(rpm)),
                packet -> {
                    outtake.set(getTargetRPM());
                    return !outtake.inRange(withinRpmRange);
                },
                new InstantAction(storage::runTransfer),
                new InstantAction(storage::openGate),
                new SleepAction(FIRE_TIME),
                new InstantAction(storage::closeGate),
                new InstantAction(outtake::stop),
                new InstantAction(storage::stopTransfer),
                new InstantAction(() -> stopAimLock())
        );
    }
}
