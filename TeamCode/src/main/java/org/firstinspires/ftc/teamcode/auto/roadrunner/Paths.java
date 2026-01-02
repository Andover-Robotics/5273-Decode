package org.firstinspires.ftc.teamcode.auto.roadrunner;

import com.acmerobotics.roadrunner.*;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

public class Paths {

    public static Action buildPath(
            MecanumDrive drive,
            BotActions actions,
            Pose2d startPose,
            Vector2d shootPos,
            Vector2d parkPos,
            Vector2d obeliskScanPos,
            Vector2d[][] artifactPositions,
            double intakeHeading,
            double outtakeHeading,
            double obeliskScanHeading
    ) {
        TrajectoryActionBuilder builder = drive.actionBuilder(startPose);
        builder.stopAndAdd(
                new RaceAction(
                        actions.actionPeriodic(), // keeps updating shooter/indexer
                        builder.fresh()
                                .build())

        );
        for (int row = 2; row >= 0; row--) {
            // Move to the row's starting Y position for intake
            builder.strafeToSplineHeading(
                    new Vector2d(0, artifactPositions[row][0].y),
                    intakeHeading
            );

            for (int col = 0; col < 3; col++) {
                builder.stopAndAdd(new ParallelAction(
                        actions.actionIntakeOneCycle(),
                        builder.fresh()
                               .strafeToSplineHeading(
                                       artifactPositions[row][col],
                                       intakeHeading
                               )
                               .build()
                ));
            }

            builder.strafeToSplineHeading(obeliskScanPos, obeliskScanHeading)
                   .stopAndAdd(actions.actionScanObelisk());

            builder.stopAndAdd(actions.actionShootWithLock(actions.aprilTag.getObeliskId(), 2.0));

            // already turns off lock-in mode in action
            // builder.stopAndAdd(new InstantAction(() -> continuousAprilTagLock = false));
        }

        return builder
                .strafeToSplineHeading(parkPos, outtakeHeading)
                .stopAndAdd(actions.actionPark())
                .build();
    }
}

