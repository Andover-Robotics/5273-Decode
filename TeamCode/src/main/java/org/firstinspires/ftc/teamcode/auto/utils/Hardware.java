package org.firstinspires.ftc.teamcode.auto.utils;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Aimer;

public class Hardware {
    public final Intake intake;
    public final Indexer indexer;
    public final Outtake outtake;
    public final Actuator actuator;
    public final AprilTag aprilTag;
    public final Aimer aprilAimer;
    public final BotActions actions;
    public final MecanumDrive mecanumDrive;

    public Hardware(HardwareMap hardwareMap, Telemetry telemetry, LinearOpMode opMode, Pose2d startPose) {
        mecanumDrive = new MecanumDrive(
                hardwareMap,
                startPose
        );

        intake   = new Intake(hardwareMap);
        indexer  = new Indexer(hardwareMap);
        indexer.AUTO_ADVANCEMENT_MODE = Indexer.AutoAdvancement.DISABLED;
        indexer.ENABLE_FULL_UNKNOWN_SCAN = false;
        indexer.SCAN_COLORS = false;

        outtake  = new Outtake(hardwareMap, Outtake.Mode.RPM);
        actuator = new Actuator(hardwareMap);
        aprilTag = new AprilTag(hardwareMap, telemetry);
        aprilAimer = new Aimer(mecanumDrive);

        actions = new BotActions(this, telemetry, opMode);
    }
}
