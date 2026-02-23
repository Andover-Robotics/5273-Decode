package org.firstinspires.ftc.teamcode.testing;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Turret;

@TeleOp(name="turret tester")
public class TurretTester extends LinearOpMode {
    @Override
    public void runOpMode() {
        Turret turret = new Turret(hardwareMap);
        waitForStart();
        GamepadEx gamepad = new GamepadEx(gamepad1);
        while (opModeIsActive()) {
            turret.setPower(gamepad.getLeftX());
            telemetry.addData("turret position", turret.getCurrentAngle());
            telemetry.update();
        }
    }
}
