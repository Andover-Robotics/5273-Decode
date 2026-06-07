package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@Config
public class Intake {
    public static double SLOW_MULTIPLIER = 0.5;
    public static double INTAKING_POWER = 1;
    private MotorEx intakeMotor;

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
    public void runSlow() {
        intakeMotor.set(INTAKING_POWER * SLOW_MULTIPLIER);
    }
    public void runBackwards()
    {
        intakeMotor.set(-INTAKING_POWER);
    }
    public void runBackwardsSlow() {
        intakeMotor.set(-INTAKING_POWER * SLOW_MULTIPLIER);
    }
    public void setPower(double newPower)
    {
        intakeMotor.set(newPower);
    }
    public double getPower()
    {
        return intakeMotor.get();
    }
    public double getCurrentAmps() {
        return intakeMotor.motorEx.getCurrent(CurrentUnit.AMPS);
    }
}
