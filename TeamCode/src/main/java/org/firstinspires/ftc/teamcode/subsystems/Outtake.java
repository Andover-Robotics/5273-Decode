package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    private final MotorEx motor;
    private double rpm;
    double lastTime;

    public Outtake(HardwareMap hardwareMap)
    {
        motor = new MotorEx(hardwareMap, "outtake");
        motor.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        lastTime = System.nanoTime();
    }

    public void stop()
    {
        motor.stopMotor();
    }

    public void run()
    {
        motor.setVelocity(rpmToTicksPerSecond(rpm));
    }

    public void setRPM(double newPower)
    {
        rpm = newPower;
    }

    public double getRpm()
    {
        return rpm;
    }

    public double getRPM()
    {
        double ticksPerSecond = motor.getVelocity();
        double ticksPerRev = motor.getCPR();
        double revsPerSecond = ticksPerSecond / ticksPerRev;
        return revsPerSecond * 60.0;
    }

    public double rpmToTicksPerSecond(double rpm) {
        double ticksPerRev = motor.getCPR(); // counts per revolution
        return (rpm / 60.0) * ticksPerRev;
    }
    //TODO: Make algorithm to determine power needed based on distance
}
