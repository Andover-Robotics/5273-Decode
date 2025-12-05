package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

@TeleOp(name = "Intake Value Tester", group = "Teleop")
public class IntakeValueTester extends LinearOpMode {
    Intake intake;
    Outtake outtake;
    GamepadEx gp1;
    GamepadEx gp2;

    @Override
    public void runOpMode(){
        gp1 = new GamepadEx(gamepad1);
        gp2 = new GamepadEx(gamepad2);
        // intake;
        outtake = new Outtake(hardwareMap, Outtake.Mode.RPM);
        boolean start = false;
        waitForStart();
        while(opModeIsActive()){
            if (!start){
                start = true;

            }
            telemetry.addData("y left",gp2.getLeftY());
            if (gp1.getButton(GamepadKeys.Button.A)){
            }
            if (gp1.getButton(GamepadKeys.Button.B)){
            }
            if (gp1.getButton(GamepadKeys.Button.Y)){
            }
            if (gp2.getButton(GamepadKeys.Button.A)){

            }
            if (gp2.getButton(GamepadKeys.Button.B)){
            }

        }
    }
}
