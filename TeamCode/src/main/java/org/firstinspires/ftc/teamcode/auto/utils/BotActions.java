package org.firstinspires.ftc.teamcode.auto.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.Aimer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

import java.util.function.IntSupplier;

@Config
public class BotActions {
    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    private final Intake intake;
    private final Indexer indexer;
    private final Outtake outtake;
    private final Actuator actuator;
    public final AprilTag aprilTag;
    private final Aimer aprilAimer;
    private final MecanumDrive drive;

    public static double NON_INDEX_SPIN_TIME = 1.35; //seconds of full-power indexer blast
    public static double FULL_BLAST_POWER =0.25;

    public static double ball1TimeDisp = 0.66;
    public static double  ball2TimeDisp = 1.10;
    public static double  timeToIntake = 2.50;

    public static boolean continuousAprilTagLock;
    public static double cooldownFeedbackIntake = 0;
    public static double quickspinRpmScale = 0.93;
    private double lastTurnCorrection;
    private int obeliskId = 0;

    public BotActions(
            Hardware hardware,
            Telemetry telemetry,
            LinearOpMode opMode
    ) {
        this.intake = hardware.intake;
        this.indexer = hardware.indexer;
        this.outtake = hardware.outtake;
        this.actuator = hardware.actuator;
        this.aprilTag = hardware.aprilTag;
        this.aprilAimer = hardware.aprilAimer;
        this.drive = hardware.mecanumDrive;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    public Action actionStartOuttake(double rpm) {
        return new ParallelAction(
            new InstantAction(() -> indexer.setAutoOuttaking(true)),
            new InstantAction(() -> outtake.set(rpm * quickspinRpmScale))
        );
    }

    public Action actionQuickOuttake() {
        return new SequentialAction(
                new InstantAction(actuator::upQuick),// lower up position for quick dump
                new SleepAction(.35),// for actuator
                new InstantAction(() -> indexer.setIndexerPower(FULL_BLAST_POWER)),// full blast
                new SleepAction(NON_INDEX_SPIN_TIME),
                new InstantAction(indexer::stopIndexerPower),
                new InstantAction(() -> indexer.setAutoOuttaking(false)),
                new InstantAction(() -> indexer.setIntaking(true, Indexer.IndexerState.two)), // This means, if you don't move the indexer, the next intaken will enter slot 0
                new InstantAction(outtake::stop),
                new InstantAction(actuator::down),
                new InstantAction(indexer::initializeColors)
        );
    }

    // helpers at the end of the file
    public Action rotateToMotifColorBeforeOuttake(int row, IntSupplier id, int startingSlot) {
        if (id.getAsInt() != 21 && id.getAsInt() != 22 && id.getAsInt() != 23) {
            return new InstantAction(() ->{});
        }

        return new InstantAction(() -> {
            // set current color configuration
            applyCurrentColorsFromRow(row, startingSlot);

            // get desired firing order from obelisk id
            Indexer.ArtifactColor[] desiredOrder = getDesiredShootOrder(id.getAsInt());

            // values gets an array of the enums
            for (Indexer.IndexerState state : Indexer.IndexerState.values()) {
                telemetry.addData("Started search for index of proper", "color");
                if (matchesOrder(state.index, desiredOrder)) {
                    telemetry.addData("Rotated To Motif", "Color");
                    // Indexer.IndexerState gotoState = Indexer.IndexerState.values()[(state.index - 1) % Indexer.IndexerState.values().length];
                    Indexer.IndexerState gotoState = state;
                    indexer.moveTo(gotoState, true);
                    return;
                }
            }
        });
    }

    public Action actionSetSomeShizzle() {
        return new SequentialAction(
                new InstantAction(() -> indexer.setAutoOuttaking(false)),
                new InstantAction(() -> indexer.setIntaking(true, Indexer.IndexerState.one)),
                new InstantAction(() -> indexer.moveTo(Indexer.IndexerState.two, true))
                );
    }

    public Action actionIntakeThree(Pose2d startActionPose, Pose2d startIntakePose, Pose2d endPose, MecanumDrive drive, double maxVel) {
        TranslationalVelConstraint velConstraint = new TranslationalVelConstraint(maxVel);

        return drive.actionBuilder(startActionPose)
                .strafeToSplineHeading(startIntakePose.position, startIntakePose.heading)
                .afterTime(0, intake::run)
                .afterTime(ball1TimeDisp, () -> indexer.moveTo(indexer.getState().next()))
                .afterTime(ball2TimeDisp, () -> indexer.moveTo(indexer.getState().next()))
                .afterTime(timeToIntake, intake::runSlow)
                .strafeToLinearHeading(endPose.position, endPose.heading, velConstraint)
                .build();
    }



    public Action initializeAuto(Indexer.IndexerState startingSlot) { // only temporary for testing, this is done in actionQuickOuttake
        return new ParallelAction(
            new SequentialAction(
                new InstantAction(() -> indexer.initializeColors(Indexer.ArtifactColor.EMPTY)),
                new InstantAction(() -> indexer.setIntaking(true, startingSlot))
            ),
            new InstantAction(actuator::down),
            new InstantAction(intake::runSlow)
        );
    }

    //feedback based version of actionIntakeT
    public Action actionIntakeThreeFeedback(
            Pose2d startActionPose,
            Pose2d startIntakePose,
            Pose2d endPose,
            MecanumDrive drive,
            double maxVel
    ) {
        VelConstraint velConstraint = new TranslationalVelConstraint(maxVel);

        Action driveAction = drive.actionBuilder(startActionPose)
                .strafeToSplineHeading(startIntakePose.position, startIntakePose.heading)
                .strafeToLinearHeading(endPose.position, endPose.heading, velConstraint)
                .build();

        Action manageIntakeAndIndexing = new Action() {
            private boolean lastAlignedNonEmpty = false;
            private int acquired = 0;
            private double timeoutDuration = 5.0;
            private final ElapsedTime acquireCooldown = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket p) {
                // stop if opmode ends
                if (!opMode.opModeIsActive() || opMode.isStopRequested()) {
                    intake.runSlow(); // or intake.stop()
                    return false;
                }

                // keep  indexer logic alive
                indexer.update();

                boolean alignedNonEmpty = indexer.artifactPresentAndAligned();

                //  intake mode continuously
                if (alignedNonEmpty) intake.runSlow(); // Lowkey the play so that once one is intaken another doesn't get stuck until indexer moves
                else intake.run();

                if (!lastAlignedNonEmpty
                        && alignedNonEmpty
                        && acquireCooldown.milliseconds() > cooldownFeedbackIntake) {

                    acquired++;
                    acquireCooldown.reset();

                    if (acquired < 3) {
                        indexer.moveTo(indexer.getState().next());
                    }
                }

                lastAlignedNonEmpty = alignedNonEmpty;

                // keep running until we've acquired 3
                if (acquired >= 3) {
                    intake.runSlow(); // or intake.stop()
                    return false;
                }

                p.put("acquired", acquired);
                p.put("alignedNonEmpty", alignedNonEmpty);
                p.put("indexerState", indexer.getState());

                return true;
            }
        };

        return new SequentialAction(
                new InstantAction(intake::run),

                new ParallelAction(
                        driveAction,
                        manageIntakeAndIndexing
                ),

                new InstantAction(intake::runSlow)
        );
    }

    // bad to do instant action and while loop
    public Action actionScanObelisk() {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!opMode.opModeIsActive() || opMode.isStopRequested()) {
                    return false;
                }

                if (timer.seconds() > 5.0) {
                    telemetry.addLine("Obelisk scan timed out");
                    return false;
                }

                aprilTag.scanObeliskTag();
                obeliskId = aprilTag.getObeliskId();

                return !(obeliskId == 21 || obeliskId == 22 || obeliskId == 23);
            }
        };
    }

    public Action actionPeriodic() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!opMode.opModeIsActive() || opMode.isStopRequested()) {
                    return false;
                }

                drive.updatePoseEstimate();
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

                return true;
            }
        };
    }

    public Action actionPark() {
    // vert slides
        return new ParallelAction(
                new SleepAction(1)
        );
    }

    public int getObeliskId() {
        return obeliskId;
    }


    private boolean matchesOrder(int stateIndex, Indexer.ArtifactColor[] desired) {
        return indexer.getColorAt(Indexer.IndexerState.values()[stateIndex % 3]) == desired[0]
                && indexer.getColorAt(Indexer.IndexerState.values()[(stateIndex + 1) % 3]) == desired[1];
    }


    // HELPERS

    // Sets the indexer's color configuration based on a given row,
    // rotated so that the first intaken ball is placed in startingSlot
    private void applyCurrentColorsFromRow(int row, int startingSlot /* basically (the last moved to slot + 1) % 3 [in intaking mode]*/) {
        Indexer.ArtifactColor[] intakeOrder;

        switch (row) {
            case 1: // P P G
                intakeOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.GREEN};
                break;

            case 2: // P G P
                intakeOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE};
                break;

            case 0:
            case 3: // G P P
                intakeOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.PURPLE};
                break;

            default:
                return;
        }

        // Rotate array so the first intaken ball lands in startingSlot
        Indexer.ArtifactColor[] rotated = new Indexer.ArtifactColor[3];
        for (int i = 0; i < 3; i++) {
            rotated[(startingSlot + i) % 3] = intakeOrder[i];
        }

        indexer.initializeColors(rotated[0], rotated[1], rotated[2]);
    }

    private Indexer.ArtifactColor[] getDesiredShootOrder(int id) {
        // After the tag ID cases, you want to change the physical rows into shooting that motif
        Indexer.ArtifactColor[] desiredShootOrder;
        switch (id) {
            case 21: // G -> P -> P - will shoot out in this order
                desiredShootOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.PURPLE};
                break;

            case 22: // P -> G -> P
                desiredShootOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.GREEN, Indexer.ArtifactColor.PURPLE};
                break;

            case 23: // P -> P -> G
                desiredShootOrder = new Indexer.ArtifactColor[] {Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.PURPLE, Indexer.ArtifactColor.GREEN};
                break;

            default:
                desiredShootOrder = new Indexer.ArtifactColor[] {};
                break;
        }

        return desiredShootOrder;
    }
}
