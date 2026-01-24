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

    public PoseMapTrajectoryBuilder waitMovement(int start, int end, double wait) {
        TrajectorySequence path = builder.build();
        double time = 0;
        for (int i = start; i <= end; i++) time += path.get(path.size() - 1 + i).getDuration();
        builder = builder.waitSeconds(Math.max(wait - time, 0));
        return this;
    }

    public PoseMapTrajectoryBuilder lineToLinearHeading(Pose2d pose, double waitTime) {
        builder = builder.lineToLinearHeading(mapPose(pose));
        return waitMovement(0, 0, waitTime);
    }

    public PoseMapTrajectoryBuilder strafeTo(Vector2d pos) {
        builder = builder.strafeTo(posMap.apply(pos));
        return this;
    }

    public TrajectorySequence build() {
        return builder.build();
    }
}
