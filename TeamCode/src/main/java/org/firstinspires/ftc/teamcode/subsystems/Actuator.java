package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Actuator {
    public static double DOWN = 0.45;      // flush with the floor of platform
    public static double UP_QUICK = 0.22; // used for quick (non-indexed) outtake
    public static double UP_INDEXED = 0.13; // used for indexed outtake\

    public enum ActuatorState
    {
        DOWN,
        UP_QUICK,
        UP_INDEXED
    }

    private ActuatorState state;
    private final SimpleServo servo;
    private double waitTime = 0.5; // seconds

    public Actuator(HardwareMap hardwareMap) {
        servo = new SimpleServo(hardwareMap, "actuator", 0, 360);
    }

    public void down() {
        servo.setPosition(DOWN);
        state = ActuatorState.DOWN;
    }

    //default up is indexed
    public void up() {
        upIndexed();
    }

    //higher position
    public void upIndexed() {
        servo.setPosition(UP_INDEXED);
        state = ActuatorState.UP_INDEXED;
    }

    //lower position
    public void upQuick() {
        servo.setPosition(UP_QUICK);
        state = ActuatorState.UP_QUICK;
    }

    public boolean isActivated() {
        return !(state == ActuatorState.DOWN);
    }

    public ActuatorState getState() {
        return state;
    }

    public void set(boolean activate) {
        if (activate) up();
        else down();
    }

    public double getWaitTime() {
        return waitTime; // seconds
    }
}