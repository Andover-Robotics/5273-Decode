package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.AnalogInput;


import org.firstinspires.ftc.teamcode.subsystems.CRServoPositionControl;

@TeleOp(name = "ContinuousServoTester")

public class ContinuousValueTester extends LinearOpMode {
  AnalogInput indexerAnalog;

      @Override
      public void runOpMode() throws InterruptedException {
          AnalogInput signal = hardwareMap.get(AnalogInput.class, "indexAnalog");
          CRServo indexerServo = hardwareMap.get(CRServo.class, "index");
          CRServoPositionControl crServo = new CRServoPositionControl(indexerServo, indexerAnalog);
          waitForStart();
          while (opModeIsActive())
          {
              indexerServo.setPower(0.1);
              telemetry.addData("voltage", signal.getVoltage());
              telemetry.update();
          }
      }
}
