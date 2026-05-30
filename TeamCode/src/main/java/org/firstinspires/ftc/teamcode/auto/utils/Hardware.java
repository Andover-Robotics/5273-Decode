package org.firstinspires.ftc.teamcode.auto.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;

public class Hardware {
    public final Intake intake;
    public final Outtake outtake;
    public final AprilTag aprilTag;
    public final Aimer aimer;
    public final BotActions actions;
    public final MecanumDrive mecanumDrive;

    public Hardware(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, Pose2d startPose) {
        mecanumDrive = new MecanumDrive(
                hardwareMap,
                startPose
        );

        intake   = new Intake(hardwareMap);

        outtake  = new Outtake(hardwareMap, Outtake.Mode.RPM);
        aprilTag = new AprilTag(hardwareMap, telemetry);
        aimer = new Aimer(mecanumDrive);

        actions = new BotActions(this, telemetry, opMode);
    }
}