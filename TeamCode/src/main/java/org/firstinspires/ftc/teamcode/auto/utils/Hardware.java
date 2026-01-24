package org.firstinspires.ftc.teamcode.auto.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTagAimer;

public class Hardware {
    public final Intake intake;
    public final Indexer indexer;
    public final Outtake outtake;
    public final Actuator actuator;
    public final AprilTag aprilTag;
    public final AprilTagAimer aprilAimer;
    public final BotActions actions;
    public final MecanumDrive mecanumDrive;
    public final IMU imu;
    public final TwoDeadWheelLocalizer deadWheelLocalizer;
    public Hardware(HardwareMap hardwareMap, Telemetry telemetry) {
        mecanumDrive = new MecanumDrive(
                hardwareMap,
                new Pose2d(0, 0, 0)
        );

        imu = mecanumDrive.lazyImu.get();
        deadWheelLocalizer = (TwoDeadWheelLocalizer) mecanumDrive.localizer;

        intake   = new Intake(hardwareMap);
        indexer  = new Indexer(hardwareMap);
        outtake  = new Outtake(hardwareMap, Outtake.Mode.RPM);
        actuator = new Actuator(hardwareMap);
        aprilTag = new AprilTag(hardwareMap, telemetry);
        aprilAimer = new AprilTagAimer(hardwareMap, imu, deadWheelLocalizer);

        actions = new BotActions(telemetry, intake, indexer, outtake, actuator, aprilTag, aprilAimer);
    }
}
