package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {

    // ARC Thunder Vortex

    private final double INTAKING_POWER = -1.0;
    private MotorEx intakeMotor;
    private boolean running;

    public Intake(HardwareMap hardwareMap)
    {
        intakeMotor = new MotorEx(hardwareMap, "intake");
    }

    public void run(boolean startMotor)
    {
        running = startMotor;
        if(startMotor)
        {
            intakeMotor.set(INTAKING_POWER);
        }
        else {
            intakeMotor.stopMotor();
        }
    }

    public boolean isRunning()
    {
        return running;
    }
}
