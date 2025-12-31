package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Actuator {
    public static double DOWN = 0.0;      // flush with the floor of platform
    public static double UP_QUICK = 0.20; // used for quick (non-indexed) outtake
    public static double UP_INDEXED = 0.34; // used for indexed outtake

    private boolean activated;

    private final SimpleServo servo;

    private double waitTime = 0.5; // seconds

    public Actuator(HardwareMap hardwareMap) {
        servo = new SimpleServo(hardwareMap, "actuator", 0, 360);
    }

    public void down() {
        servo.setPosition(DOWN);
        activated = false;
    }

    //default up is indexed
    public void up() {
        upIndexed();
    }

    //highe rposition
    public void upIndexed() {
        servo.setPosition(UP_INDEXED);
        activated = true;
    }

    //lower position
    public void upQuick() {
        servo.setPosition(UP_QUICK);
        activated = true;
    }

    public boolean isActivated() { return activated; }

    public void set(boolean activate) {
        if (activate) up();
        else down();
    }

    public double getWaitTime() {
        return waitTime; // seconds
    }
}