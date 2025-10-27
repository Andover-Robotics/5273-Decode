package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@TeleOp(name = "Servo Value Finder", group = "Teleop")
public class TriggerIntakeActuatorIndexerTester extends LinearOpMode {
    Intake intake;
    Outtake outtake;
    GamepadEx gp1;
    GamepadEx gp2;
    Actuator actuator;
    Indexer indexer;

    @Override
    public void runOpMode(){
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        intake = new Intake(hardwareMap);
        outtake = new Outtake(hardwareMap);
        actuator = new Actuator(hardwareMap);
        indexer = new Indexer(hardwareMap);

        boolean triggerPressed = false;
        waitForStart();
        while (opModeIsActive()) {
            if (gp1.wasJustPressed(GamepadKeys.Button.A)) {
                intake.run(!intake.isRunning());
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.B)) {
                actuator.set(!actuator.isActivated());
            }
            if (gp1.wasJustPressed(GamepadKeys.Button.DPAD_RIGHT)) {
                indexer.moveTo(indexer.nextState());
            }
            if (!triggerPressed && gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5) {
                triggerPressed = true;
                indexer.quickSpin();
            } else if (triggerPressed && gp1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) < 0.1) {
                triggerPressed = false;
            }
        }
    }
}
