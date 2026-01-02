package org.firstinspires.ftc.teamcode.auto.roadrunner.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.auto.roadrunner.Paths;
import org.firstinspires.ftc.teamcode.auto.roadrunner.Hardware;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

import com.acmerobotics.dashboard.config.Config;

@Config
@Autonomous(name = "Red-Close", group = "Autonomous")
public class RedClose extends LinearOpMode {

    // START POSE
    public static double startX = 0;
    public static double startY = 0;
    public static double startHeadingDeg = 180;

    // SHOOT POSITION
    public static double shootX = -10;
    public static double shootY = 10;

    // PARK POSITION
    public static double parkX = 0;
    public static double parkY = 100;

    // OBELISK SCAN POSITION
    public static double obeliskScanX = 0;
    public static double obeliskScanY = 50;

    // HEADINGS
    public static double intakeHeadingDeg = 180;
    public static double outtakeHeadingDeg = 135;
    public static double obeliskScanHeadingDeg = 90;

    // ARTIFACTS
    public static double[][] artifactX = {
            {-20, -25, -30},  // Row 0
            {-20, -25, -30},  // Row 1
            {-20, -25, -30}   // Row 2
    };
    public static double[][] artifactY = {
            {48, 48, 48},     // Row 0
            {72, 72, 72},     // Row 1
            {96, 96, 96}      // Row 2
    };

    @Override
    public void runOpMode() {
        // Initialize hardware and drive
        Hardware hardware = new Hardware(hardwareMap, telemetry);

        // Convert dashboard fields to Pose2d/Vector2d
        Pose2d startPose = new Pose2d(startX, startY, Math.toRadians(startHeadingDeg));
        Vector2d shootPos = new Vector2d(shootX, shootY);
        Vector2d parkPos = new Vector2d(parkX, parkY);
        Vector2d obeliskScanPos = new Vector2d(obeliskScanX, obeliskScanY);
        double intakeHeading = Math.toRadians(intakeHeadingDeg);
        double outtakeHeading = Math.toRadians(outtakeHeadingDeg);
        double obeliskScanHeading = Math.toRadians(obeliskScanHeadingDeg);

        // Build artifacts array from dashboard values
        Vector2d[][] artifacts = new Vector2d[3][3];
        for (int row = 0; row < 3; row++) {
            for (int col = 0; col < 3; col++) {
                artifacts[row][col] = new Vector2d(artifactX[row][col], artifactY[row][col]);
            }
        }

        // Build autonomous path
        Action auto = Paths.buildPath(
                hardware.mecanumDrive,
                hardware.actions,
                startPose,
                shootPos,
                parkPos,
                obeliskScanPos,
                artifacts,
                intakeHeading,
                outtakeHeading,
                obeliskScanHeading
        );

        waitForStart();
        Actions.runBlocking(auto);
    }
}
