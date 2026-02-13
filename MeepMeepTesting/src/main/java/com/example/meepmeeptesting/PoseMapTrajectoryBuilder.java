package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequence;
import org.rowlandhall.meepmeep.roadrunner.trajectorysequence.TrajectorySequenceBuilder;

import java.util.function.Function;

public class PoseMapTrajectoryBuilder {
    private TrajectorySequenceBuilder builder;
    private final Function<Vector2d, Vector2d> posMap;
    private final Function<Double, Double> headingMap;
    public PoseMapTrajectoryBuilder(
            Function<Pose2d, TrajectorySequenceBuilder> realBuilder,
            Pose2d start,
            Function<Vector2d, Vector2d> posMap,
            Function<Double, Double> headingMap
    ) {
        this.posMap = posMap;
        this.headingMap = headingMap;
        builder = realBuilder.apply(mapPose(start));
    }

    private Pose2d mapPose(Pose2d pose) {
        return new Pose2d(
                posMap.apply(pose.vec()),
                headingMap.apply(pose.component3())
        );
    }

    public PoseMapTrajectoryBuilder lineToLinearHeading(Pose2d pose) {
        builder = builder.lineToLinearHeading(mapPose(pose));
        return this;
    }

    public PoseMapTrajectoryBuilder lineToSplineHeading(Pose2d pose) {
        builder = builder.lineToSplineHeading(mapPose(pose));
        return this;
    }

    public TrajectorySequence build() {
        return builder.build();
    }
}
