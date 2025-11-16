package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Actuator {
    private final int DOWN = 0; // flush with the floor of platform
    private final int UP = 180; // raised to push it into the flywheel

    private boolean activated;

    private final SimpleServo servo;

    public Actuator(HardwareMap hardwareMap) {
        servo = new SimpleServo(hardwareMap, "actuator", 0, 360);
    }

    public void down() {
        servo.turnToAngle(DOWN);
        activated = false;
    }

    public void up() {
        servo.turnToAngle(UP);
        activated = true;
    }

    public boolean isActivated()
    {
        return activated;
    }
}
