package org.firstinspires.ftc.teamcode.auto.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Arclength;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PosePath;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.VelConstraint;
import com.arcrobotics.ftclib.trajectory.constraint.TrajectoryConstraint;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Actuator;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTagAimer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@Config
public class BotActions {
    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    private final Intake intake;
    private final Indexer indexer;
    private final Outtake outtake;
    private final Actuator actuator;
    public final AprilTag aprilTag;
    private final AprilTagAimer aprilAimer;

    public static double NON_INDEX_SPIN_TIME = 1.35; //seconds of full-power indexer blast
    public static double FULL_BLAST_POWER =0.25;

    public static double ball1TimeDisp = 0.66;
    public static double  ball2TimeDisp = 1.10;
    public static double  timeToIntake = 2.50;

    public static boolean continuousAprilTagLock;
    private double lastTurnCorrection;

    public BotActions(
            Telemetry telemetry,
            Intake intake,
            Indexer indexer,
            Outtake outtake,
            Actuator actuator,
            AprilTag aprilTag,
            AprilTagAimer aprilAimer,
            LinearOpMode opMode
    ) {
        this.intake = intake;
        this.indexer = indexer;
        this.outtake = outtake;
        this.actuator = actuator;
        this.aprilTag = aprilTag;
        this.aprilAimer = aprilAimer;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    public Action actionStartOuttake(double rpm) {
        return new ParallelAction(
            new InstantAction(() -> indexer.setAutoOuttaking(true)),
            new InstantAction(() -> outtake.set(rpm))
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
    public Action rotateToMotifColorBeforeOuttake(int row, int id, int startingSlot) {
        return new InstantAction(() -> {
            // set current color configuration
            applyCurrentColorsFromRow(row, startingSlot);

            // get desired firing order from obelisk id
            Indexer.ArtifactColor[] desiredOrder = getDesiredShootOrder(id);

            // values gets an array of the enums
            for (Indexer.IndexerState state : Indexer.IndexerState.values()) {
                telemetry.addData("Started search for index of proper", "color");
                if (matchesOrder(state.index, desiredOrder)) {
                    telemetry.addData("Rotated To Motif", "Color");
                    // Indexer.IndexerState gotoState = Indexer.IndexerState.values()[(state.index - 1) % Indexer.IndexerState.values().length];
                    Indexer.IndexerState gotoState = state;
                    indexer.moveTo(gotoState);
                    return;
                }
            }
        });
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
                        && acquireCooldown.milliseconds() > 150) {

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

    public Action initializeAuto(Indexer.IndexerState startingSlot) { // only temporary for testing, this is done in actionQuickOuttake
        return new ParallelAction(
            new SequentialAction(
                new InstantAction(() -> indexer.initializeColors(Indexer.ArtifactColor.EMPTY)),
                new InstantAction(() -> indexer.setIntaking(true, Indexer.IndexerState.two))
            ),
            new InstantAction(actuator::down),
            new InstantAction(intake::runSlow)
        );
    }

    // bad to do instant action and while loop
    public Action actionScanObelisk() {
        return new Action() {
            private int scannedId = -1;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!opMode.opModeIsActive() || opMode.isStopRequested()) {
                    return false;
                }

                if (scannedId != 21 && scannedId != 22 && scannedId != 23) {
                    aprilTag.scanObeliskTag();
                    scannedId = aprilTag.getObeliskId();
                    return true;
                }
                else {
                    aprilTag.setCurrentCameraScannedId(scannedId);
                    return false;
                }
            }
        };
    }

    // doesn't seem to work with parallel actions
    // Oops return false tells it to stop true tells it to go which is why it prob didn't work before?
    public Action actionPeriodic() {
        return new Action() {
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!opMode.opModeIsActive() || opMode.isStopRequested()) {
                    return false;
                }

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
        return aprilTag.getObeliskId();
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
