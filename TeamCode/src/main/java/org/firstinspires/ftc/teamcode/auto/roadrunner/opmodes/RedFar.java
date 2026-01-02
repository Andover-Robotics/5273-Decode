package org.firstinspires.ftc.teamcode.auto.roadrunner.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.*;
import org.firstinspires.ftc.teamcode.auto.roadrunner.Paths;
import org.firstinspires.ftc.teamcode.auto.roadrunner.Hardware;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

@Autonomous(name = "Red-Far", group = "Autonomous")
public class RedFar extends LinearOpMode {

    @Override
    public void runOpMode() {
        // Initialize hardware and drive
        Hardware hardware = new Hardware(hardwareMap, telemetry);
        MecanumDrive drive = new MecanumDrive(
                hardwareMap,
                new Pose2d(0, 0, Math.toRadians(180))
        );

        // Define positions and headings
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        Vector2d shootPos = new Vector2d(-10, 10);
        Vector2d parkPos = new Vector2d(0, 100);
        Vector2d obeliskScanPos = new Vector2d(0, 50);
        double intakeHeading = Math.toRadians(180);
        double outtakeHeading = Math.toRadians(135);
        double obeliskScanHeading = Math.toRadians(90);

        // Define artifact positions
        Vector2d[][] artifacts = new Vector2d[3][3];
        artifacts[2][0] = new Vector2d(-20, 96);
        artifacts[2][1] = new Vector2d(-25, 96);
        artifacts[2][2] = new Vector2d(-30, 96);
        artifacts[1][0] = new Vector2d(-20, 72);
        artifacts[1][1] = new Vector2d(-25, 72);
        artifacts[1][2] = new Vector2d(-30, 72);
        artifacts[0][0] = new Vector2d(-20, 48);
        artifacts[0][1] = new Vector2d(-25, 48);
        artifacts[0][2] = new Vector2d(-30, 48);

        // Build autonomous path
        Action auto = Paths.buildPath(
                drive,
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
    }
}
