package org.firstinspires.ftc.teamcode.auto.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;

public class Hardware {
    public final Intake intake;
    public final Storage storage;
    public final Outtake outtake;
    public final Turret turret;
    public final AprilTag aprilTag;
    public final Aimer aimer;
    public final MecanumDrive mecanumDrive;

    public Hardware(HardwareMap hardwareMap, Telemetry telemetry, Pose2d startPose, boolean isFarShooting) {
        mecanumDrive = new MecanumDrive(
                hardwareMap,
                startPose
        );

        intake   = new Intake(hardwareMap);
        storage = new Storage(hardwareMap, intake);
        outtake  = new Outtake(hardwareMap, Outtake.Mode.RPM);
        turret = new Turret(hardwareMap);
        aprilTag = new AprilTag(hardwareMap, telemetry);
        aimer = new Aimer(mecanumDrive, isFarShooting);
    }
}