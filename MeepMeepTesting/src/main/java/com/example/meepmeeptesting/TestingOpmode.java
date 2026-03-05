package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.roadrunner.DriveShim;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;

public class TestingOpmode {
    public static double maxIntakeDrivingVel = 17;

    // OLD NON SPLINES
    public static double OBELISK_X = 8;
    public static double OBELISK_Y = 36;
    public static double OBELISK_HEADING_DEG = 65;

    public static double SHOOT_X = 17;
    public static double SHOOT_Y = 40;
    public static double SHOOT_HEADING_DEG = -142.5;
    public static double SHOOT_HEADING_OFFSET_AFTER_FIRSTSHOT = 0;

    public static double INTAKE_START_X = 12;
    public static double INTAKE2_START_OFFSET_X = 3.0;
    public static double INTAKE3_START_OFFSET_X = 5.0;
    public static double INTAKE_END_X = -16;
    public static double intake_END_2And3_XOffset = 6.0;

    public static double INTAKE1_Y = 50;
    public static double INTAKE2_Y = 76.5;
    public static double INTAKE3_Y = 96;

    public static double gate_X = -12;
    public static double gate_Y = 60;

    public static double gateWaitTime = 1.5;

    public static double PARK_X = 6;
    public static double PARK_Y = 68;

    public static int SHOOT_RPM = 3240;

    public static double timeUntilStartOuttake = 1.65; // Time until you start the outtake action, which still includes the wait for actuator


    // NEW SPLINES

    // has quick outtake and quick intake
    public static TrajectorySequence createPath(DriveShim drive) {
        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));

        Pose2d shootingPose = new Pose2d(
                SHOOT_X,
                SHOOT_Y,
                Math.toRadians(SHOOT_HEADING_DEG)
        );

        Pose2d intake1PoseStart = new Pose2d(INTAKE_START_X, INTAKE1_Y, Math.toRadians(180));
        Pose2d intake1PoseEnd = new Pose2d(INTAKE_END_X, INTAKE1_Y, Math.toRadians(180));
        Pose2d gate = new Pose2d(gate_X, gate_Y, Math.toRadians(-90));

        Pose2d intake2PoseStart = new Pose2d(INTAKE_START_X + INTAKE2_START_OFFSET_X, INTAKE2_Y, Math.toRadians(180));
        Pose2d intake2PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE2_Y, Math.toRadians(180));
        Pose2d dodgeGate = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset + 14, INTAKE2_Y - 4, Math.toRadians(140));

        Pose2d intake3PoseStart = new Pose2d(INTAKE_START_X + INTAKE3_START_OFFSET_X, INTAKE3_Y, Math.toRadians(180));
        Pose2d intake3PoseEnd = new Pose2d(INTAKE_END_X - intake_END_2And3_XOffset, INTAKE3_Y, Math.toRadians(180));

        Pose2d parkPose = new Pose2d(PARK_X, PARK_Y, Math.toRadians(180));

        // Later combine this with toShoot for smoother
        Pose2d obeliskPose = new Pose2d(OBELISK_X, OBELISK_Y, OBELISK_HEADING_DEG);


        return new PoseMapTrajectoryBuilder(
                drive::trajectorySequenceBuilder,
                startPose,
                pos -> new Vector2d(pos.getY() - 62.5, 35 - pos.getX()),
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
