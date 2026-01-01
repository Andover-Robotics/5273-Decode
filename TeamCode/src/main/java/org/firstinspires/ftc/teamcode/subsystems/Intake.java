package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Intake {

    // ARC Thunder Vortex

    public static double INTAKING_POWER = -1;
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
