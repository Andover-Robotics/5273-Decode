package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class FasterBlueCloseSimpleMotif {

    public static double OBELISK_X = -8;
    public static double OBELISK_Y = 36;
    public static double OBELISK_HEADING_DEG = -120;

    public static double SHOOT_X = -12;
    public static double SHOOT_Y = 42;
    public static double SHOOT_HEADING_DEG = -50;

    public static double INTAKE_X = -8;

    public static double gate_Y = 63;
    public static double INTAKE1_Y = 51;
    public static double INTAKE2_Y = 75;
    public static double INTAKE3_Y = 99;
    public static double INTAKE_FORWARD_DIST = 8;

    public static double PARK_X = -6;
    public static double PARK_Y = 68;

    public static int SHOOT_RPM = 4010;

    public static double timeUntilStartIntake = 1.5; // has to be very accurate, subject to issues depending on voltage
    public static double timeUntilStartOuttake = 1.0; // Time until you start the outtake action, which still includes the spinup time

    // has quick outtake and quick intake
    public static TrajectorySequence createPath(DriveShim drive) {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

        Pose2d obeliskPose = new Pose2d(
                OBELISK_X,
                OBELISK_Y,
                Math.toRadians(OBELISK_HEADING_DEG)
        );

        Pose2d shootingPose = new Pose2d(
                SHOOT_X,
                SHOOT_Y,
                Math.toRadians(SHOOT_HEADING_DEG)
        );

        Pose2d intake1PoseStart = new Pose2d(INTAKE_X, INTAKE1_Y, Math.toRadians(0));
        Pose2d intake1PoseEnd = new Pose2d(INTAKE_X + 3 * INTAKE_FORWARD_DIST + 4, INTAKE1_Y, Math.toRadians(0));
        Pose2d gate = new Pose2d(INTAKE_X + 3 * INTAKE_FORWARD_DIST + 4, gate_Y, Math.toRadians(-90));

        Pose2d intake2PoseStart = new Pose2d(INTAKE_X, INTAKE2_Y, Math.toRadians(0));
        Pose2d intake2PoseEnd = new Pose2d(INTAKE_X + 3 * INTAKE_FORWARD_DIST + 4.5, INTAKE2_Y, Math.toRadians(0));
        Pose2d dodgeGate = new Pose2d(INTAKE_X + 2 * INTAKE_FORWARD_DIST, INTAKE2_Y - 2, Math.toRadians(-20));

        Pose2d intake3PoseStart = new Pose2d(INTAKE_X, INTAKE3_Y, Math.toRadians(0));
        Pose2d intake3PoseEnd = new Pose2d(INTAKE_X + 3 * INTAKE_FORWARD_DIST + 4.5, INTAKE3_Y, Math.toRadians(0));

        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(0));

        return new PoseMapTrajectoryBuilder(
                drive::trajectorySequenceBuilder,
                startPose,
                pos -> new Vector2d(pos.getY() - 62.5, -35 - pos.getX()),
                heading -> heading - Math.toRadians(90)
        )
                .lineToSplineHeading(obeliskPose)
                .lineToSplineHeading(shootingPose)
                .lineToSplineHeading(intake1PoseStart)
                .lineToLinearHeading(intake1PoseEnd)
                .lineToSplineHeading(shootingPose)
                .lineToSplineHeading(intake2PoseStart)
                .lineToLinearHeading(intake2PoseEnd)
                .lineToSplineHeading(dodgeGate)
                .lineToSplineHeading(shootingPose)
                .lineToSplineHeading(intake3PoseStart)
                .lineToLinearHeading(intake3PoseEnd)
                .lineToSplineHeading(shootingPose)
                .lineToSplineHeading(parkPose)
                .build();
    }
}