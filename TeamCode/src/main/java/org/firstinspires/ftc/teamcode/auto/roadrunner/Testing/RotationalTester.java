package org.firstinspires.ftc.teamcode.auto.roadrunner.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;



@Config
public class RotationalTester extends OpMode {
    public static double TARGET_DEG = 90.0;
    public static double TARGET_RATE_DEG = 180;
    double goalRad;   // where we eventually want to go


    public static double HEADING_GAIN;        // kP
    public static double HEADING_VEL_GAIN ;    // kD

    public static double kS;
    public static double kV;
    public static double kA;
    // === Drive ===
    MecanumDrive drive;

    // === Dashboard ===
    FtcDashboard dashboard;
    double lastHeading;
    double lastVel = 0.0;
    long lastTime;
    double targetRad;

    public static double SETTLE_ERROR_DEG = 2.0;
    public static double SETTLE_VEL_DEG = 5.0;
    @Override
    public void init() {
        // Hardware setup
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        targetRad = drive.localizer.getPose().heading.toDouble();
        HEADING_GAIN = MecanumDrive.PARAMS.headingGain;
        HEADING_VEL_GAIN = MecanumDrive.PARAMS.headingVelGain;
        kS = MecanumDrive.PARAMS.kS;
        kV = MecanumDrive.PARAMS.kV;
        kA = MecanumDrive.PARAMS.kA;
        lastTime = System.nanoTime();
        lastHeading = targetRad;
        goalRad =targetRad;
        // Dashboard telemetry
        dashboard = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
    }


    @Override
    public void loop() {
        drive.updatePoseEstimate();
        Pose2d pose = drive.localizer.getPose();
        double heading = pose.heading.toDouble();

        long now = System.nanoTime();
        double dt = (now - lastTime) * 1e-9;
        lastTime = now;
        if (dt <= 0) return;

        double maxStep = Math.toRadians(TARGET_RATE_DEG) * dt;
        double diff = wrapHeading(targetRad - goalRad);
        diff = clamp(diff, -maxStep, maxStep);
        goalRad = wrapHeading(goalRad + diff);
        // --- Error ---
        double error = wrapHeading(goalRad - heading);

        // --- Velocity & accel ---
        double vel = (heading - lastHeading) / dt;
        double accel = (vel - lastVel) / dt;

        lastHeading = heading;
        lastVel = vel;

        // --- Feedforward ---
        double ff =
                Math.signum(vel) * kS +
                        kV * vel +
                        kA * accel;

        // --- Feedback ---
        double fb =
                HEADING_GAIN * error -
                        HEADING_VEL_GAIN * vel;

        double turnPower = ff + fb;
        turnPower = Math.max(-1.0, Math.min(1.0, turnPower));
        drive.setDrivePowers(new PoseVelocity2d( new Vector2d(0,0), turnPower));

        // --- Advance target when settled ---
        boolean settled =
                Math.abs(Math.toDegrees(error)) < SETTLE_ERROR_DEG &&
                        Math.abs(Math.toDegrees(vel)) < SETTLE_VEL_DEG;

        if (settled) {
            targetRad += Math.toRadians(TARGET_DEG);
        }
        // --- Telemetry ---
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("Target (deg)", Math.toDegrees(targetRad));
        telemetry.addData("Goal (deg)", Math.toDegrees(goalRad));
        telemetry.addData("Error (deg)", Math.toDegrees(error));
        telemetry.addData("Angular vel (deg/s)", Math.toDegrees(vel));
        telemetry.addData("Turn power", turnPower);
        telemetry.update();
    }
    private double wrapHeading(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

}
