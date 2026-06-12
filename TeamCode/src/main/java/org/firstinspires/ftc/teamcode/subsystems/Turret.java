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
    public static double actualRangeOfMotion = 322.0;
    public static double servoOffset = 86.0;


    public Turret(HardwareMap hardwareMap) {
        servo1 = new SimpleServo(hardwareMap, "turretServo1", 0, 360);
        servo2 = new SimpleServo(hardwareMap, "turretServo2", 0, 360);
        // set initial position
        servo1.setPosition(0);
        servo2.setPosition(0);
    }

    public double getCurrentAngle() {
        return angle;
    }

    public void rotate(double angle) {
        /*        0.5
                1.0 0.0
         when angle is 0 degrees + servoOffset, servo sets to 0.5
         when angle is 180 degrees + servoOffset, servo sets to 0.0
         when angle is -180 degrees + servoOffset, servo sets to 1.0

         The spot where the turret needs to do a 360 is in the back
        */

        // normalize heading error to servo's 0 to 1, negate angle based on whether turret is clockwise or counterclockwise from 0 to 1
        double targetAngle = wrapAngle360(-angle + 180 + servoOffset);
        double physicalRangeCenter = actualRangeOfMotion / 2;
        double relativeToCenter = angleWrapNegPos180(targetAngle - physicalRangeCenter);

        if (Math.abs(relativeToCenter) > physicalRangeCenter) {
            double deadZone = 360 - actualRangeOfMotion;

            if (relativeToCenter > 0) {
                targetAngle = actualRangeOfMotion;
            }
            else {
                targetAngle = 0;
            }
        }

        double servoPos = targetAngle / actualRangeOfMotion;
        servo1.setPosition(servoPos);
        servo2.setPosition(servoPos);
    }

    private double wrapAngle360(double angle) {
        return ((angle%360)+360)%360;
    }
    private double angleWrapNegPos180(double angle) {
        return ((angle + 180) % 360 + 360) % 360 - 180;
    }


    public void setServos(double angle) {
        servo1.setPosition(angle / actualRangeOfMotion);
        servo2.setPosition(angle / actualRangeOfMotion);
    }

    public double getServoOffset() {
        return servoOffset;
    }

    public double getActualRangeOfMotion() {
        return actualRangeOfMotion;
    }

    // used for testing
    public void setServo1(double angle) {
        servo1.setPosition(angle / actualRangeOfMotion);
    }
    public void setServo2(double angle) {
        servo2.setPosition(angle / actualRangeOfMotion);
    }
}