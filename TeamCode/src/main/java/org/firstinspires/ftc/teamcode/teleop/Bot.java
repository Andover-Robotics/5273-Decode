package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Aimer;

@Config
public class Bot extends BotPeriodics {
    // haptics & lights
    private boolean finishedInitialGateClose = false;
    private long teleOpStartTime;
    public static long timeToReverseTransferAfterStartBeforeCloseGate = 500; // ms

    public static double withinRpmRange = 150; //
    public static double FIRE_TIME = 1.0;

    public static Pose2d startPose = new Pose2d(0, 0, 0);

    public Bot(HardwareMap hardwareMap, Telemetry tele, MecanumDrive mecanumDrive, Gamepad gamepad1, Gamepad gamepad2, boolean twoMovement, boolean isFarShooting) {
        super(hardwareMap, tele, mecanumDrive, gamepad1, gamepad2, twoMovement, isFarShooting);
        hardwareMap.voltageSensor.iterator().next().getVoltage(); // ensure voltage sensor is initialized before teleop starts
    }

    public void teleopInit() {
        outtake.stop();
    }

    public void teleopStart(){
        teleOpStartTime = System.currentTimeMillis();
        initialBackwardsTransfer = true;
        turret.initialize();
        aimer.localize(startPose);
        if (goal == Aimer.Goal.BLUE) {
            aimer.setBlueTarget();
        }
        else {
            aimer.setRedTarget();
        }
    }

    public void teleopTick(boolean isFarShooting)
    {
        if (!finishedInitialGateClose && System.currentTimeMillis() >= teleOpStartTime + timeToReverseTransferAfterStartBeforeCloseGate) {
            storage.closeGate();
            initialBackwardsTransfer = false;
            finishedInitialGateClose = true;
        }

        handlePeriodics();
        handleIntakeFeedback();
        handleOuttakeActions();
    }

    // MAINLINE HANDLERS
    private void handleIntakeFeedback() {
        if(storage.isFull() && !turnCurrentSensingOff){
            if (!manualTurretAim)
                setAimlock(true);

            if (!rumbledAlready) {
                g1.gamepad.rumbleBlips(TeleopConstants.Gamepad.FULL_WARNING_RUMBLES);
                g2.gamepad.rumbleBlips(TeleopConstants.Gamepad.FULL_WARNING_RUMBLES);

                setRumbledAlready(true);
            }
        }
    }

    private void handleOuttakeActions() {
        if (!actionHost.isRunning() && g2.wasJustPressed(GamepadKeys.Button.A) && aimlock) {
            actionHost.start(actionFire(isFarShooting));
        }
    }

    @Override
    protected void handleAllianceSelection() {
        if (g1.wasJustPressed(GamepadKeys.Button.BACK) || g2.wasJustPressed(GamepadKeys.Button.BACK)) {
            aimer.setBlueTarget();
            g1.gamepad.setLedColor(0, 0, 1, TeleopConstants.Gamepad.GAMEPAD_LIGHT_COLOR_DURATION);
            goal = Aimer.Goal.RED;
        }
        if (g1.wasJustPressed(GamepadKeys.Button.START) || g2.wasJustPressed(GamepadKeys.Button.START)) {
            aimer.setRedTarget();
            g1.gamepad.setLedColor(1, 0, 0, TeleopConstants.Gamepad.GAMEPAD_LIGHT_COLOR_DURATION);
            goal = Aimer.Goal.RED;
        }
    }

    private Action actionFire(boolean isFarShooting) {
        Action shootingAction = new SequentialAction(
                packet -> {
                    outtake.set(getTargetRPM());
                    return !outtake.inRange(withinRpmRange, 75);
                },
                new InstantAction(intake::run),
                new InstantAction(() -> storage.runTransfer(isFarShooting)),
                new InstantAction(storage::openGate),
                new SleepAction(FIRE_TIME),
                new InstantAction(storage::closeGate),
                new InstantAction(intake::stop),
                new InstantAction(outtake::stop),
                new InstantAction(storage::stopTransfer),
                new InstantAction(() -> setAimlock(false)),
                new InstantAction(() -> setRumbledAlready(false)),
                new InstantAction(() -> setMecanumAvoiding(false))
        );

        return new ParallelAction(
                shootingAction,
                packet -> {
                    // Packet returns true when shootingAction ends
                    outtake.set(getTargetRPM());
                    return shootingAction.run(packet);
                }
        );
    }
}
