package org.firstinspires.ftc.teamcode.auto.roadrunner;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;

public class Hardware {
    public final Intake intake;
    public final Indexer indexer;
    public final Outtake outtake;
    public final Actuator actuator;
    public final AprilTag aprilTag;
    public final AprilTagAimer aprilAimer;
    public final BotActions actions;

    public Hardware(HardwareMap hardwareMap, Telemetry tele) {
        intake   = new Intake(hardwareMap);
        indexer  = new Indexer(hardwareMap);
        outtake  = new Outtake(hardwareMap, Outtake.Mode.RPM);
        actuator = new Actuator(hardwareMap);
        aprilTag = new AprilTag(hardwareMap);
        aprilAimer = new AprilTag(hardwareMap);

        actions = new BotActions(intake, indexer, outtake, actuator, aprilTag, aprilAimer);
    }
}
