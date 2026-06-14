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
    private final Intake intake;

    public static double gateClosedPos = 0.345;
    public static double gateOpenPos = 0.067;
    public static double TRANSFER_POWER = -0.7;
    public static double fullDurationThreshold = 300; // ms
    public static double fullCurrentThresholdAmps = 8.0; // amps
    private ElapsedTime fullDurationTimer;
    private double    current = 0; // amps

    private static boolean gateOpen = false;
    public Storage (HardwareMap hardwareMap, Intake intake){
        transfer = new MotorEx(hardwareMap, "transfer");
        gate = new SimpleServo(hardwareMap, "gate", 0, 360);
        fullDurationTimer = new ElapsedTime();
        this.intake = intake;
    }
    public void openGate()
    {
        gate.setPosition(gateOpenPos);
        gateOpen = true;
    }
    public void closeGate() {
        gate.setPosition(gateClosedPos);
        gateOpen = false;

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
        current = intake.getCurrentAmps() + transfer.motorEx.getCurrent(CurrentUnit.AMPS);
        if (!(current >= fullCurrentThresholdAmps)) {
            fullDurationTimer.reset();
        }
    }

    public boolean isFull() {
        return current >= fullCurrentThresholdAmps && fullDurationTimer.milliseconds() >= fullDurationThreshold;
    }

    public double getCurrent() {
        return current;
    }
}
