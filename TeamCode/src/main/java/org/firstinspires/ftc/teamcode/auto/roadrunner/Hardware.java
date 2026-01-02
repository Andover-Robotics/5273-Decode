package org.firstinspires.ftc.teamcode.auto.roadrunner;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.TwoDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class Hardware {
    public final Intake intake;
    public final Indexer indexer;
    public final Outtake outtake;
    public final Actuator actuator;
    public final AprilTag aprilTag;
    public final AprilTagAimer aprilAimer;
    public final BotActions actions;
    public final IMU imu;
    public final TwoDeadWheelLocalizer deadWheelLocalizer;
    public Hardware(HardwareMap hardwareMap, Telemetry telemetry) {
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));

        imu.initialize(parameters);

        deadWheelLocalizer =
                new TwoDeadWheelLocalizer(
                        hardwareMap,
                        imu,
                        0.00195844,
                        new Pose2d(0, 0, 0)
                );
        intake   = new Intake(hardwareMap);
        indexer  = new Indexer(hardwareMap);
        outtake  = new Outtake(hardwareMap, Outtake.Mode.RPM);
        actuator = new Actuator(hardwareMap);
        aprilTag = new AprilTag(hardwareMap, telemetry);
        aprilAimer = new AprilTagAimer(hardwareMap, imu, deadWheelLocalizer);

        actions = new BotActions(intake, indexer, outtake, actuator, aprilTag, aprilAimer);
    }
}
