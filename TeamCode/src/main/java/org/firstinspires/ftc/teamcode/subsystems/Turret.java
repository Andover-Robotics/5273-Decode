package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTagAimer;

public class Turret {
    private static double ticksToDegrees = 0;

    private AprilTagAimer aprilTagAimer;
    private DcMotor motor;
    private double motorBearing = 0;
    private int lastMotorTick = 0;

    public Turret(HardwareMap hardwareMap, IMU imu, TwoDeadWheelLocalizer deadWheelLocalizer) {
        aprilTagAimer = new AprilTagAimer(hardwareMap,imu,deadWheelLocalizer);
        motor = hardwareMap.get(DcMotor.class, "TurretMotor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public double[] calculateLocalizedTurretBearing(int tagID) {
        return aprilTagAimer.calculateLocalizedBearing(tagID);
    }

    public double calculateTurnPowerFromBearing(double cameraBearing) {
        return aprilTagAimer.calculateTurnPowerFromBearing(cameraBearing - motorBearing);
    }

    public void rotateTurret(double power) {
        motor.setPower(power);
        int currentTick = motor.getCurrentPosition();
        motorBearing += (currentTick - lastMotorTick) * ticksToDegrees;
        lastMotorTick = currentTick;
    }
}