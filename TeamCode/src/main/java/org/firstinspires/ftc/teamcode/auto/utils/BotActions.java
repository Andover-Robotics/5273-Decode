package org.firstinspires.ftc.teamcode.auto.utils;

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
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.teleop.Bot;


public class BotActions {
    private final Telemetry telemetry;
    private final Intake intake;
    private final Indexer indexer;
    private final Outtake outtake;
    private final Actuator actuator;
    public final AprilTag aprilTag;
    private final AprilTagAimer aprilAimer;

    public static double NON_INDEX_SPIN_TIME = 3;//seconds of full-power indexer blast
    public static double SHOOTER_SPINUP = 2.0;
    public static double FULL_BLAST_POWER =0.25;

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

    public Action actionQuickOuttake(int rpm) {
        return new SequentialAction(
                new InstantAction(actuator::upQuick),// lower up position for quick dump
                new InstantAction(() -> outtake.set(rpm)),
                new SleepAction(SHOOTER_SPINUP),                      // spin up shooter
                new InstantAction(() -> indexer.setIndexerPower(FULL_BLAST_POWER)),// full blast
                new SleepAction(NON_INDEX_SPIN_TIME),
                new InstantAction(indexer::stopIndexerPower),
                new InstantAction(outtake::stop),
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.setIntaking(true)),
                new InstantAction(indexer::initializeColors),
                new InstantAction(() -> indexer.moveTo(Indexer.IndexerState.zero))
        );
    }

    // very slow ver
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
                new SleepAction(1.6),
                new InstantAction(actuator::upIndexed),
                new SleepAction(0.2),
                new InstantAction(actuator::down),
                new SleepAction(0.6),

                // 2nd
                new InstantAction(() -> indexer.moveTo(indexer.getState().next(), true)),
                new SleepAction(0.9),
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
    public  Action indexerRotateForMotif(int tagId, int row) {
        int rotations = 0;

        if (row == 0 || row == 1) { // G P P
            switch (tagId) {
                case 21: rotations = 2; break;
                case 22: rotations = 1; break;
                case 23: rotations = 0; break;
            }
        } else if (row == 2) { // P G P
            switch (tagId) {
                case 21: rotations = 1; break;
                case 22: rotations = 0; break;
                case 23: rotations = 2; break;
            }
        }

        switch (rotations) {
            case 2:
                return new SequentialAction(
                        actionIndexerNext(),
                        actionIndexerNext()
                );
            case 1:
                return actionIndexerNext();
            default:
                return new Action() {
                    @Override public boolean run(@NonNull com.acmerobotics.dashboard.telemetry.TelemetryPacket p) {
                        return true;
                    }
                };
        }
    }

    // Separate to run while moving
    public Action actionOuttakeOffsetForMotif(int tagID, int row) {

        int offset = 0;

        switch (row) {
            // Row 0 & 1 intake: P P G
            case 0:
            case 1:
                switch (tagID) {
                    case 21: // G P P
                        offset = 0;
                        break;
                    case 22: // P G P
                        offset = 1;
                        break;
                    case 23: // P P G
                        offset = 2;
                        break;
                }
                break;


            // Row 2 intake: P G P
            case 2:
                switch (tagID) {
                    case 21: // G P P
                        offset = 1;
                        break;
                    case 22: // P G P
                        offset = 2;
                        break;
                    case 23: // P P G
                        offset = 0;
                        break;
                }
                break;

            // Row 3 intake: G P P
            case 3:
                switch (tagID) {
                    case 21: // G P P
                        offset = 2;
                        break;
                    case 22: // P G P
                        offset = 0;
                        break;
                    case 23: // P P G
                        offset = 1;
                        break;
                }
                break;
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

    /*
    public Action actionMotifOffsetWithColorApi(int tagID, int row) {

        switch (row) {
            // Row 0 & 1 intake: P P G
            case 0:
            case 1:
                switch (tagID) {
                    case 21: // G P P
                        initializeColors();
                        break;
                    case 22: // P G P
                        initializeColors();
                        break;
                    case 23: // P P G
                        initializeColors();
                        break;
                }
                break;


            // Row 2 intake: P G P
            case 2:
                switch (tagID) {
                    case 21: // G P P
                        initializeColors();
                        break;
                    case 22: // P G P
                        initializeColors();
                        break;
                    case 23: // P P G
                        initializeColors();
                        break;
                }
                break;

            // Row 3 intake: G P P
            case 3:
                switch (tagID) {
                    case 21: // G P P
                        initializeColors();
                        break;
                    case 22: // P G P
                        initializeColors();
                        break;
                    case 23: // P P G
                        initializeColors();
                        break;
                }
                break;
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
    */

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

    public Action initializeForIntake(Indexer.IndexerState slot) {
        return new SequentialAction(
            new InstantAction(() -> indexer.setIntaking(true)),
            new InstantAction(() -> indexer.moveTo(slot, true))
        );
    }

    public Action actionIntakeThreeFast() {
        return new SequentialAction(
                new InstantAction(intake::stop),
                // slot 1
                new SleepAction(0.3),
                new InstantAction(() -> indexer.moveTo(indexer.getState().next())),

                // slot 2
                new SleepAction(0.2),
                new InstantAction(() -> indexer.moveTo(indexer.getState().next())),

                // slot 3
                new SleepAction(0.2),
                new InstantAction(() -> indexer.moveTo(indexer.getState().next())),

                // stop intakeintake
                new SleepAction(0.15),
                new InstantAction(intake::stop)
        );
    }

    // for now to fix issues
    public Action actionIndexerNext() {
        return new InstantAction(() -> {
                indexer.moveTo(indexer.getState().next());
        });
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
