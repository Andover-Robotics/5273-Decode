package org.firstinspires.ftc.teamcode.auto.roadrunner.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

@Config
public class RotationalTester extends OpMode {

    // === Drive ===
    MecanumDrive drive;

    // === Dashboard ===
    FtcDashboard dashboard;

    @Override
    public void init() {
        // Hardware setup
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        // Dashboard telemetry
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }


    @Override
    public void loop() {
        // Safety: only run while the OpMode is active
       // if (!opModeIsActive()) return;

        // Update Road Runner pose
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();

        // Send telemetry
        telemetry.addData("X", pose.position.x);
        telemetry.addData("Y", pose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
        telemetry.update();
    }
}
