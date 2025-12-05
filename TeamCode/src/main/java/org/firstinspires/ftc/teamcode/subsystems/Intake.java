package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    // ARC Thunder Vortex

    private final double INTAKING_POWER = -0.8;
    private MotorEx intakeMotor;
    private boolean running;

    public Intake(HardwareMap hardwareMap)
    {
        intakeMotor = new MotorEx(hardwareMap, "intake");
    }

    public void stop()
    {
        intakeMotor.stopMotor();
    }

    public void run()
    {
        intakeMotor.set(INTAKING_POWER);
    }

    public void setPower(double newPower)
    {
        intakeMotor.set(INTAKING_POWER);
    }
}
