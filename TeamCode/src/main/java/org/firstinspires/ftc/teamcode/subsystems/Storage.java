package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Storage {
    private final MotorEx transfer;
    private final SimpleServo gate;
    public static double gateClosedPos = 0.2;
    public static double gateOpenPos = 0;
    public static double TRANSFER_POWER = -1;
    public static double fullDurationThreshold = 1000; // ms
    public static double fullCurrentThreshold = 0; // amps
    private ElapsedTime fullDurationTimer;
    private double current = 0; // amps

    private static boolean gateOpen = false;
    public Storage (HardwareMap hardwareMap){
        transfer = new MotorEx(hardwareMap, "transfer");
        gate = new SimpleServo(hardwareMap, "gate", 0, 360);
        fullDurationTimer = new ElapsedTime();
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

    public void updateForIfFull() {
        current = transfer.motorEx.getCurrent(CurrentUnit.AMPS);
        if (!(current >= fullCurrentThreshold)) {
            fullDurationTimer.reset();
        }
    }

    public boolean isFull() {
        return current >= fullCurrentThreshold && fullDurationTimer.milliseconds() >= fullDurationThreshold;
    }

    public double getCurrent() {
        return current;
    }
}
