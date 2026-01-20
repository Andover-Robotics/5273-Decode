package org.firstinspires.ftc.teamcode.auto.roadrunner.Testing;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.TwoDeadWheelLocalizer;


public class StrafeTester extends LinearOpMode {
    public static double DISTANCE = 72;

    @Override
    public void runOpMode() throws InterruptedException {

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        if (drive.localizer instanceof TwoDeadWheelLocalizer) {
            if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
            if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                throw new RuntimeException("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
            }
        }
        waitForStart();
        // Track last pose for per-tick error
        Pose2d lastPose = drive.localizer.getPose();
        TelemetryPacket packet = new TelemetryPacket();

        Action action = drive.actionBuilder(new Pose2d(0, 0, 0))
                .lineToY(DISTANCE)
                .lineToY(0)
                .build();

        while (opModeIsActive() && action.run(packet)) {
            drive.updatePoseEstimate();
            Pose2d currentPose = drive.localizer.getPose();

            // Calculate per-tick delta
            double dx = currentPose.position.x - lastPose.position.x;
            double dy = currentPose.position.y - lastPose.position.y;
            double dHeading = currentPose.heading.toDouble() - lastPose.heading.toDouble();

            // Desired per-tick delta
            double desiredDx = 0; // strafing, x shouldn't change
            double desiredDy = (DISTANCE - lastPose.position.y); // remaining distance to target
            double desiredDHeading = 0; // heading should stay constant

            // Compute instantaneous error
            double errorX = desiredDx - dx;
            double errorY = desiredDy - dy;
            double errorHeading = dHeading - desiredDHeading;

            // Update lastPose for next tick
            lastPose = currentPose;

            // Add telemetry
            packet.put("Error X", errorX);
            packet.put("Error Y", errorY);
            packet.put("Error Heading (rad)", errorHeading);
            packet.put("X", currentPose.position.x);
            packet.put("Y", currentPose.position.y);
            packet.put("Heading (deg)", Math.toDegrees(currentPose.heading.toDouble()));

            telemetry.addData("X", currentPose.position.x);
            telemetry.addData("Y", currentPose.position.y);
            telemetry.addData("Heading (deg)", Math.toDegrees(currentPose.heading.toDouble()));
            telemetry.addData("Error X", errorX);
            telemetry.addData("Error Y", errorY);
            telemetry.update();

        }



    }
    /*
    private double wrapHeading(double radians) {
        while (radians > Math.PI) radians -= 2.0 * Math.PI;
        while (radians < -Math.PI) radians += 2.0 * Math.PI;
        return radians;
    }
    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }

     */
}