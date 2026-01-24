package org.firstinspires.ftc.teamcode.auto.helpers;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Vector2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;


public class BotActions {
    private final Telemetry telemetry;
    private final Intake intake;
    private final Indexer indexer;
    private final Outtake outtake;
    private final Actuator actuator;
    public final AprilTag aprilTag;
    private final AprilTagAimer aprilAimer;
    private int obeliskId;
    public static double NON_INDEX_SPIN_TIME = 6;//seconds of full-power indexer blast
    public static double SHOOTER_SPINUP = 3.0;
    public static double FULL_BLAST_POWER =0.6;
    public static boolean continuousAprilTagLock;
    private double lastTurnCorrection;

    public BotActions(
            Telemetry telemetry,
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
        this.telemetry = telemetry;
    }

    public void initializeColors(Indexer.ArtifactColor one, Indexer.ArtifactColor two, Indexer.ArtifactColor three) {
        indexer.initializeColors(one, two, three);
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

    public Action actionOuttake(int rpm) {
        return new SequentialAction(
                new InstantAction(() -> indexer.setIntaking(false)),
                new InstantAction(actuator::down),

                new Action() {
                    private long startTime = -1;
                    @Override
                    public boolean run(@NonNull TelemetryPacket telemetry) {
                        if (startTime < 0) startTime = System.currentTimeMillis();
                        outtake.set(rpm);
                        return System.currentTimeMillis() - startTime >= BotActions.SHOOTER_SPINUP * 1000;
                    }
                },

                // 1st
                // This movestate is needed to make sure its outtake in the same order its intaken(unless changed elsewhere)
                new InstantAction(() -> indexer.moveTo(indexer.getState().next(), true)),
                new SleepAction(1.8),
                new InstantAction(actuator::upIndexed),
                new SleepAction(0.2),
                new InstantAction(actuator::down),
                new SleepAction(0.6),

                // 2nd
                new InstantAction(() -> indexer.moveTo(indexer.getState().next(), true)),
                new SleepAction(0.8),
                new InstantAction(actuator::upIndexed),
                new SleepAction(0.2),
                new InstantAction(actuator::down),
                new SleepAction(0.6),

                // 3rd
                new InstantAction(() -> indexer.moveTo(indexer.getState().next(), true)),
                new SleepAction(0.8),
                new InstantAction(actuator::upIndexed),
                new SleepAction(0.15),
                new InstantAction(actuator::down),

                // Finish
                new InstantAction(() -> {
                    actuator.down();
                    outtake.stop();
                    indexer.setIntaking(true);
                })
        );
    }

    // Separate to run while moving
    public Action actionOuttakeOffsetForMotif(int tagID, int row) {

        int offset = 0;

        switch (row) {

            // row 0 & 1: PPG
            case 0:
            case 1:
                switch (tagID) {
                    case 21: // GPP -> rotate 2 times
                        offset = 2;
                        break;
                    case 22: // PGP -> rotate 1 times
                        offset = 1;
                        break;
                    case 23: // PPG -> no rotate
                        offset = 0;
                        break;
                    default:
                        return new InstantAction(() -> {
                        });
                }
                break;

            // row 2: PGP
            case 2:
                switch (tagID) {
                    case 21: // GPP -> rotate 1 times
                        offset = 1;
                        break;
                    case 22: // PGP -> no rotate
                        offset = 0;
                        break;
                    case 23: // PPG -> rotate 2 times
                        offset = 2;
                        break;
                    default:
                        return new InstantAction(() -> {
                        });
                }
                break;

            // row 3: GPP
            case 3:
                switch (tagID) {
                    case 21: // GPP -> no rotate
                        offset = 1;
                        break;
                    case 22: // P G P -> rotate 2 times
                        offset = 0;
                        break;
                    case 23: // P P G -> rotate 1 times
                        offset = 2;
                        break;
                    default:
                        return new InstantAction(() -> {
                        });
                }
        }

                return new SequentialAction(
                        offset >= 1
                                ? new InstantAction(() ->
                                indexer.moveTo(indexer.getState().next(), true))
                                : new InstantAction(() -> {
                        }),

                        // rotate second time if true
                        offset >= 2
                                ? new InstantAction(() ->
                                indexer.moveTo(indexer.getState().next(), true))
                                : new InstantAction(() -> {
                        })
                );
    }


    public Action actionOuttakeWithColor(int tagID, int rpm) {
        switch (tagID) {
            case 21:
                return new SequentialAction(
                        actionFireGreen(rpm),
                        actionFirePurple(rpm),
                        actionFirePurple(rpm)
                );
            case 22:
                return new SequentialAction(
                        actionFirePurple(rpm),
                        actionFireGreen(rpm),
                        actionFirePurple(rpm)
                );
            case 23:
                return new SequentialAction(
                        actionFirePurple(rpm),
                        actionFirePurple(rpm),
                        actionFireGreen(rpm)
                );
            default:
                return new InstantAction(() -> {}); // do nothing if invalid
        }
    }

    // locks in for 1 sec, then runs actionOuttake while locked in, when that finishes stops locking in
    public Action actionShootWithLock(int tagID, double shootDuration, MecanumDrive mecanumDrive) {
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

                mecanumDrive.setDrivePowers(
                        new PoseVelocity2d(new Vector2d(0, 0), turnCorrection)
                );

                // Set shooter RPM based on distance
                int shooterRPM = 0;
                if (lastTurnCorrection != 0 && !Double.isNaN(lastTurnCorrection)) {
                    shooterRPM = (int) outtake.getRegressionRPM(aprilTag.getRange());
                }

                actionOuttakeWithColor(tagID, shooterRPM).run(telemetryPacket);

                if (now - startTime >= shootDuration * 1000) {
                    continuousAprilTagLock = false;
                    return true;
                }

                return false;
            }
        };
    }

    // should probably not do instant action and while loop but it works, maybe change
    public Action actionScanObelisk() {
        return new InstantAction(() -> {
            int scannedId = -1; // Keep scanning until we get a valid obelisk ID
            while (scannedId < 21 || scannedId > 23) {
                aprilTag.scanObeliskTag();
                scannedId = aprilTag.getObeliskId();
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


    public Action actionIntakeOneCycle(boolean moveIndexer) {
        return new SequentialAction(
                new InstantAction(() -> {
                    indexer.setIntaking(true);
                    intake.run();
                }),
                new SleepAction(0.9),
                // Only move the indexer if moveIndexer is true
                new InstantAction(() -> {
                    if (moveIndexer) {
                        indexer.moveTo(indexer.getState().next());
                    }
                }),
                new SleepAction(0.167),
                new InstantAction(intake::stop)
        );
    }


    // doesn't seem to work with parallel actions
    public Action actionPeriodic() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                outtake.periodic();
                indexer.update();

                if (continuousAprilTagLock) {
                    aprilTag.scanGoalTag();
                    double bearing = aprilTag.getBearing();

                    double turn = 0;
                    if (!Double.isNaN(bearing)) {
                        turn = aprilAimer.calculateTurnPowerFromBearing(bearing);
                    }


                }

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
