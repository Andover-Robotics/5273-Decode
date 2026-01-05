package org.firstinspires.ftc.teamcode.auto.helpers;

import com.acmerobotics.roadrunner.*;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;

public class Paths {

    public static Action buildPath(
            MecanumDrive mecanumDrive,
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
        //actions.initializeColors(Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.PURPLE);

        TrajectoryActionBuilder builder = mecanumDrive.actionBuilder(startPose);
        /*builder.stopAndAdd(
                new RaceAction(
                        actions.actionPeriodic(), // keeps updating shooter/indexer
                        builder.fresh()
                                .build())

        );*/

        builder.strafeToSplineHeading(obeliskScanPos, obeliskScanHeading)
                /*.stopAndAdd(actions.actionScanObelisk())*/;
        builder.strafeToSplineHeading(shootPos, outtakeHeading)
               /*.stopAndAdd(actions.actionShootWithLock(actions.aprilTag.getObeliskId(), 2.0, mecanumDrive))*/;

        for (int row = 2; row >= 0; row--) {
            // Move to the row's starting position for intake
            builder.strafeToSplineHeading(
                    artifactPositions[row][0],
                    intakeHeading
            );

            for (int col = 1; col <= 3; col++) {
                builder/*.stopAndAdd(new ParallelAction(
                        //actions.actionIntakeOneCycle(),
                //        mecanumDrive.actionBuilder(new Pose2d(artifactPositions[row][0], intakeHeading))
                                */.strafeToSplineHeading(
                                       artifactPositions[row][col],
                                       intakeHeading
                               )
                        /*)
                )*/;
                //builder.afterTime(.5, actions.actionIntakeOneCycle());
            }

            //builder.stopAndAdd(actions.actionShootWithLock(actions.aprilTag.getObeliskId(), 2.0, mecanumDrive));

            // already turns off lock-in mode in action
            // builder.stopAndAdd(new InstantAction(() -> continuousAprilTagLock = false));
        }

        return builder
                .strafeToSplineHeading(parkPos, outtakeHeading)
                //.stopAndAdd(actions.actionPark())
                .build();
    }
}

