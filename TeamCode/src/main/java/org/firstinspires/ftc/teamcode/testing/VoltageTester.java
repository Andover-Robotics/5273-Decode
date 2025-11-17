package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@TeleOp(name = "Voltage Tester")
public class VoltageTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Indexer indexer = new Indexer(hardwareMap);
        waitForStart();
        while (opModeIsActive())
        {
            telemetry.addData("voltage", indexer.getVoltageAnalog());
            telemetry.update();
        }
    }
}
