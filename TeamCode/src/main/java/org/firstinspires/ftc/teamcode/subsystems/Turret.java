package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTagAimer;

public class Turret {
    private static final double TICKS_TO_DEGREES = 0;
    private static final double ZERO_OFFSET = 0;
    private final DcMotor motor;

    public Turret(HardwareMap hardwareMap, IMU imu, TwoDeadWheelLocalizer deadWheelLocalizer) {
        motor = hardwareMap.get(DcMotor.class, "TurretMotor");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // May be needed?
        // motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public double getCurrentAngle() {
        double curAngle = motor.getCurrentPosition() * TICKS_TO_DEGREES + ZERO_OFFSET;
        // In Java -2 % 5 = -2, not 3
        return ((curAngle % 360) + 360) % 360;
    }
    public void setPower(double power) {
        motor.setPower(power);
    }
}