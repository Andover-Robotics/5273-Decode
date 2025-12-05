package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config

public class Actuator {
    public final double DOWN = 0.0; // flush with the floor of platform
    public final double UP = .17; // raised to push it into the flywheel

    private boolean activated;

    private final SimpleServo servo;

    public Actuator(HardwareMap hardwareMap) {
        servo = new SimpleServo(hardwareMap, "actuator", 0, 360);
    }

    public void down() {
        servo.setPosition(DOWN);
        activated = false;
    }

    public void up() {
        servo.setPosition(UP);
        activated = true;
    }

    public boolean isActivated()
    {
        return activated;
    }

    public void set(boolean activate)
    {
        if (activate)
            up();
        else
            down();
    }
}