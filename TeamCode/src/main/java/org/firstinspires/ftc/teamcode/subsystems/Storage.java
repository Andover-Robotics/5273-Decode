package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Storage {
    private final MotorEx transfer;
    private final SimpleServo gate;
    public static double gateClosedPos = 0.2;
    public static double gateOpenPos = 0;
    public static double TRANSFER_POWER = -1;
    public static double TRANSFER_POWER_STALL = -0.5;
    private static boolean gateOpen = false;
    public Storage (HardwareMap hardwareMap){
        transfer = new MotorEx(hardwareMap, "transfer");
        gate = new SimpleServo(hardwareMap, "gate", 0, 360);
    }
    public void openGate()
    {
        gate.setPosition(gateOpenPos);
        gateOpen = true;
    }
    public void closeGate() {
        gate.setPosition(gateClosedPos);
        gateOpen = true;

    }

    public void runTransfer() {
        transfer.set(TRANSFER_POWER);
    }

    public void stopTransfer() {
        transfer.stopMotor();
    }

    public void runTransferBackwards() {
        transfer.set(-TRANSFER_POWER);
    }

    public boolean isFull() {
        /* Probably voltage based detection with intake*/
        return false;
    }
}
