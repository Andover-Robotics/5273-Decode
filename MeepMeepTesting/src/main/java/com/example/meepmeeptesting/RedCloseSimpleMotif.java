package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class RedCloseSimpleMotif {

    public static double SHOOT_X = 12;
    public static double SHOOT_Y = 42;
    public static double SHOOT_HEADING_DEG =225;

    public static double INTAKE_X = 10;
    public static double INTAKE1_Y = 51;
    public static double INTAKE2_Y = 75;
    public static double INTAKE_FORWARD_DIST = 8;

    public static double PARK_X = 6;
    public static double PARK_Y = 68;

    public static TrajectorySequence createPath(DriveShim drive) {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

        Pose2d shootingPose = new Pose2d(
                SHOOT_X,
                SHOOT_Y,
                Math.toRadians(SHOOT_HEADING_DEG)
        );

        Pose2d intake1PoseStart = new Pose2d(INTAKE_X, INTAKE1_Y, Math.toRadians(180));
        Pose2d intake1Pose1 = new Pose2d(INTAKE_X - INTAKE_FORWARD_DIST, INTAKE1_Y, Math.toRadians(180));
        Pose2d intake1Pose2 = new Pose2d(INTAKE_X - 2 * INTAKE_FORWARD_DIST, INTAKE1_Y, Math.toRadians(180));
        Pose2d intake1Pose3 = new Pose2d(INTAKE_X - 3 * INTAKE_FORWARD_DIST - 4, INTAKE1_Y, Math.toRadians(180));

        Pose2d intake2PoseStart = new Pose2d(INTAKE_X, INTAKE2_Y, Math.toRadians(180));
        Pose2d intake2Pose1 = new Pose2d(INTAKE_X - INTAKE_FORWARD_DIST, INTAKE2_Y, Math.toRadians(180));
        Pose2d intake2Pose2 = new Pose2d(INTAKE_X - 2 * INTAKE_FORWARD_DIST, INTAKE2_Y, Math.toRadians(180));
        Pose2d intake2Pose3 = new Pose2d(INTAKE_X - 3 * INTAKE_FORWARD_DIST - 4.5, INTAKE2_Y, Math.toRadians(180));
        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(180));

        return new PoseMapTrajectoryBuilder(
                drive::trajectorySequenceBuilder,
                startPose,
                pos -> new Vector2d(pos.getY() - 62.5, 35 - pos.getX()),
                heading -> heading - Math.toRadians(90)
        )
                .lineToLinearHeading(shootingPose, Time.ACTION_QUICK_OUTTAKE_TIME)
                .lineToLinearHeading(intake1PoseStart)
                .lineToLinearHeading(intake1Pose1, Time.ACTION_INTAKE_ONE_CYCLE_TIME)
                .lineToLinearHeading(intake1Pose2, Time.ACTION_INTAKE_ONE_CYCLE_TIME)
                .lineToLinearHeading(intake1Pose3, Time.ACTION_INTAKE_ONE_CYCLE_TIME)
                .lineToLinearHeading(shootingPose, Time.BACK_TO_SHOOT_TIME)
                .lineToLinearHeading(intake2PoseStart)
                .lineToLinearHeading(intake2Pose1, Time.ACTION_INTAKE_ONE_CYCLE_TIME)
                .lineToLinearHeading(intake2Pose2, Time.ACTION_INTAKE_ONE_CYCLE_TIME)
                .lineToLinearHeading(intake2Pose3, Time.ACTION_INTAKE_ONE_CYCLE_TIME)
                .strafeTo(new Vector2d(INTAKE_X + 3 * INTAKE_FORWARD_DIST, INTAKE2_Y))
                .lineToLinearHeading(shootingPose)
                .waitMovement(-1, 0, Time.BACK_TO_SHOOT_TIME)
                .lineToLinearHeading(parkPose)
                .build();
    }
}