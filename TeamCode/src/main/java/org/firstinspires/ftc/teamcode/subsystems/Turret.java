package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Turret {
    private final SimpleServo servo1;
    private final SimpleServo servo2;
    private double angle = -67.0;


    public Turret(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(SimpleServo.class, "turretServo1");
        servo2 = hardwareMap.get(SimpleServo.class, "turretServo2");
        // set initial position
        servo1.setPosition(0);
        servo2.setPosition(0);
    }

    public double getCurrentAngle() {return angle;}
    public void rotate(double angle) {
        this.angle = angle;
        double position = (angle / 355);
       servo1.setPosition(position);
       servo2.setPosition(position);
    }
}