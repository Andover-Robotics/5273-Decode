package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Storage {
    public DcMotor transfer;
    public Servo gate;
    public double gateClosedPos = 0;
    public double gateOpenPos = 1;
    public double transferStall = 0.5;
    public double transferShoot = 1;
    public Storage (HardwareMap hardwareMap){
        transfer = hardwareMap.get(DcMotor.class, "transfer");
        gate = hardwareMap.get(Servo.class, "gate");
    }
    public void closeGate()
    {
        as
    }
}
