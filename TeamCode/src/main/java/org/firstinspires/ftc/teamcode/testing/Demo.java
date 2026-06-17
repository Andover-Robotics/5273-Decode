package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleop.BotPeriodics;
import org.firstinspires.ftc.teamcode.teleop.TeleopConstants;

@TeleOp(name = "Demo", group = "AA_main")
public class Demo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DemoBot bot = new DemoBot(hardwareMap, telemetry, new MecanumDrive(hardwareMap, new Pose2d(0, 0, Math.toRadians(0))), gamepad1, gamepad2,false);
        bot.teleopInit();
        waitForStart();
        bot.teleopStart();
        while (opModeIsActive() && !isStopRequested()) {
            bot.teleopTick();
        }
    }
}

class DemoBot extends BotPeriodics {
    public static double NON_INDEX_SPIN_TIME = 3; //seconds of full-power indexer blast
    public static double SHOOTER_SPINUP = 2.0;
    public static double FULL_BLAST_POWER = 0.25;
    public static double QUICKSPIN_OUTTAKE_RPM_SCALE = 0.94; // 1.12
    // Press-and-hold pre-spin minimum RPM
    public static double INTAKE_MIN_RPM = 3500.0;
    public static double RPM =  1800.0;
    public static boolean isFarShooting = false;

    public DemoBot(HardwareMap hardwareMap, Telemetry tele, MecanumDrive drive, Gamepad gamepad1, Gamepad gamepad2, boolean twoMovement) {
        super(hardwareMap, tele, drive, gamepad1, gamepad2, twoMovement);
    }

    public void teleopInit() {
        outtake.stop();
    }

    public void teleopStart(){
        storage.closeGate();
    }

    public void teleopTick()
    {
        handlePeriodics(isFarShooting);
        handleDemo();
    }

    // MAINLINE HANDLERS
    private void handleDemo() {
        double leftTrigger = g2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER);
        //double rightTrigger = g2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER);

        // Press-and-hold right bumper to spin up shooter while in Intake
        if (g2.gamepad.right_bumper) {
            applyPreSpinRPM();
        } else {
            outtake.stop();
        }

        if (leftTrigger > TeleopConstants.Gamepad.TRIGGER_DEADZONE) intake.run();
        else intake.stop();

        if (g2.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT));

        if(g2.wasJustPressed(GamepadKeys.Button.DPAD_UP)){

        }

        if(g2.wasJustPressed(GamepadKeys.Button.X)){
        }
    }

    private void applyPreSpinRPM() {
        outtake.set(RPM); // RPM mode: set shooter target RPM
    }
}
