package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

public class AntiDefense {

    public static double kP = 0.0;
    public static double kH = 0.0; //heading gain
    public static double xyTolerance = 0.5;
    public static double aTolerance = Math.PI /12;
    MecanumDrive drive;
    private Pose2d referencePoint;
    public AntiDefense(Telemetry telemetry, MecanumDrive drive)
    {
        this.drive = drive;
    }
    public void lock(Pose2d currentBotPos)
    {
        referencePoint = currentBotPos;
    }
    public void unlock()
    {
        referencePoint = null;
    }
    public void update(Pose2d currentPos)
    {
        if(referencePoint == null) return;
        Pose2d error = referencePoint.minusExp(currentPos);
        //double
        Vector2d xyError = error.position;
        double theta = error.heading.toDouble();

        if(xyError.norm() < xyTolerance && Math.abs(theta) < aTolerance) return;
        drive.setDrivePowers(new PoseVelocity2d(error.position.times(kP), theta * kH));
    }
}
