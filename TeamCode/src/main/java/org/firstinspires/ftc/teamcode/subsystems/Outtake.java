package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    private final MotorEx motor;
    private double power;
    final double TICKS_PER_REV = 112.0;

    public Outtake(HardwareMap hardwareMap)
    {
        motor = new MotorEx(hardwareMap, "outtake");

    }

    public void stop()
    {
        motor.stopMotor();
    }

    public void run()
    {
        motor.set(power);
    }

    public void setPower(double newPower)
    {
        power = newPower;
        motor.set(power);
    }

    public double getPower()
    {
        return power;
    }

    //TODO: Make algorithm to determine power needed based on distance
}
