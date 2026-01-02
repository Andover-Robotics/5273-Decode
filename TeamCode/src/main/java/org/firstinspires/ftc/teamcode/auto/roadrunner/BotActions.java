/*
 * Action methods for use in Auto
 */

package org.firstinspires.ftc.teamcode.auto.roadrunner;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;

import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;


public class BotActions {
    private final Intake intake;
    private final Indexer indexer;
    private final Outtake outtake;
    private final Actuator actuator;
    public final AprilTag aprilTag;
    private final AprilTagAimer aprilAimer;
    private int obeliskId;
    public static double NON_INDEX_SPIN_TIME = 6;//seconds of full-power indexer blast
    public static double SHOOTER_SPINUP = 2.0;
    public static double FULL_BLAST_POWER =0.6;
    public static boolean continuousAprilTagLock;
    private double lastTurnCorrection;

    public BotActions(
            Intake intake,
            Indexer indexer,
            Outtake outtake,
            Actuator actuator,
            AprilTag aprilTag,
            AprilTagAimer aprilAimer
    ) {
        this.intake = intake;
        this.indexer = indexer;
        this.outtake = outtake;
        this.actuator = actuator;
        this.aprilTag = aprilTag;
        this.aprilAimer = aprilAimer;
    }

    public Action actionNonIndexedDump(
            double rpm,
            double spinupTime,
            double blastTime,
            double blastPower
    ) {
        return new SequentialAction(
                new InstantAction(actuator::upQuick),
                new InstantAction(() -> outtake.set(rpm)),
                new SleepAction(spinupTime),
                new InstantAction(() -> indexer.setIndexerPower(blastPower)),
                new SleepAction(blastTime),
                new InstantAction(indexer::stopIndexerPower),
                new InstantAction(outtake::stop),
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.setIntaking(true)),
                new InstantAction(indexer::initializeColors),
                new InstantAction(() -> indexer.moveTo(Indexer.IndexerState.zero))
        );
    }

    public Action actionOuttake(int tagID, int rpm) {
        switch (tagID) {
            case 21:
                return new SequentialAction(
                        new InstantAction(() -> actionFireGreen(rpm)),
                        new InstantAction(() -> actionFirePurple(rpm)),
                        new InstantAction(() -> actionFirePurple(rpm))
                );
            case 22:
                return new SequentialAction(
                        new InstantAction(() -> actionFirePurple(rpm)),
                        new InstantAction(() -> actionFireGreen(rpm)),
                        new InstantAction(() -> actionFirePurple(rpm))
                );
            case 23:
                return new SequentialAction(
                        new InstantAction(() -> actionFirePurple(rpm)),
                        new InstantAction(() -> actionFirePurple(rpm)),
                        new InstantAction(() -> actionFireGreen(rpm))
                );
            default:
                return new InstantAction(() -> {}); // do nothing if invalid
        }
    }

    // locks in for 1 sec, then runs actionOuttake while locked in, when that finishes stops locking in
    public Action actionShootWithLock(int tagID, double shootDuration) {
        return new Action() {
            private long startTime = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                long now = System.currentTimeMillis();

                if (startTime < 0) {
                    startTime = now;
                    continuousAprilTagLock = true; // turn on lock mode
                }

                // Update Limelight aiming continuously
                aprilTag.scanGoalTag();
                double bearing = aprilTag.getBearing();
                lastTurnCorrection = !Double.isNaN(bearing)
                        ? aprilAimer.calculateTurnPowerFromBearing(bearing)
                        : 0;
                double turnCorrection = 0.9 * lastTurnCorrection;

                // Set shooter RPM based on distance
                int shooterRPM = (int) outtake.getRegressionRPM(aprilTag.getRange());
                actionOuttake(tagID, shooterRPM).run(telemetryPacket);

                if (now - startTime >= shootDuration * 1000) {
                    continuousAprilTagLock = false;
                    return true;
                }

                return false;
            }
        };
    }

    public Action actionScanObelisk() {
        return new InstantAction(() -> {
            int scannedId = -1;

            // Keep scanning until we get a valid obelisk ID
            while (scannedId < 21 || scannedId > 23) {
                aprilTag.scanObeliskTag();
                scannedId = aprilTag.getObeliskId();

                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    break;
                }
            }

            aprilTag.setCurrentCameraScannedId(scannedId);
        });
    }

    public Action actionFireGreen(int rpm) {
        final Indexer.IndexerState slot =
                indexer.findBestSlotForColor(Indexer.ArtifactColor.GREEN);

        if (slot == null) {
            return new InstantAction(() -> {});
        }

        return new SequentialAction(
                new InstantAction(() -> indexer.setIntaking(false)),
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.moveTo(slot, true)),
                new InstantAction(() -> outtake.set(rpm)),
                new SleepAction(SHOOTER_SPINUP),
                new InstantAction(actuator::upIndexed),
                new SleepAction(1),

                new InstantAction(() ->
                        indexer.assignSlotColor(slot, Indexer.ArtifactColor.EMPTY)
                ),

                new InstantAction(outtake::stop),
                new InstantAction(actuator::down)
        );
    }

    public Action actionFirePurple(int rpm) {
        final Indexer.IndexerState slot =
                indexer.findBestSlotForColor(Indexer.ArtifactColor.PURPLE);

        if (slot == null) {
            return new InstantAction(() -> {});
        }

        return new SequentialAction(
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.moveTo(slot, true)),

                new InstantAction(() -> outtake.set(rpm)),
                new SleepAction(SHOOTER_SPINUP),
                new InstantAction(actuator::upIndexed),
                new SleepAction(1),

                new InstantAction(() ->
                        indexer.assignSlotColor(slot, Indexer.ArtifactColor.EMPTY)
                ),

                new InstantAction(outtake::stop),
                new InstantAction(actuator::down)
        );
    }


    public Action actionIntakeOneCycle() {
        return new SequentialAction(
                new InstantAction(() -> {
                    intake.run();
                    indexer.setIntaking(true);
                }),
                new SleepAction(0.67),
                new InstantAction(intake::stop),
                new InstantAction(() -> indexer.moveTo(indexer.getState().next()))
        );
    }

    public Action actionPeriodic() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                outtake.periodic(); // PIDF shooter update
                indexer.update(); // PIDF indexer update
                return false;
            }
        };
    }


    public Action actionPark() {
    // vert slides
        return new ParallelAction(
                new SleepAction(1)
        );
    }
}
