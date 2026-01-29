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
    private final Telemetry telemetry;
    private final Intake intake;
    private final Indexer indexer;
    private final Outtake outtake;
    private final Actuator actuator;
    public final AprilTag aprilTag;
    private final AprilTagAimer aprilAimer;

    public static double NON_INDEX_SPIN_TIME = 1.35; //seconds of full-power indexer blast
    public static double FULL_BLAST_POWER =0.25;

    public static double ball1TimeDisp = 0.33;
    public static double  ball2TimeDisp = 0.545;
    public static double  timeToIntake = 1.75;

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

    public Action actionStartOuttake(double rpm) {
        return new InstantAction(() -> outtake.set(rpm));
    }

    public Action actionQuickOuttake() {
        return new SequentialAction(
                new InstantAction(actuator::upQuick),// lower up position for quick dump
                new SleepAction(.35),// for actuator
                new InstantAction(() -> indexer.setIndexerPower(FULL_BLAST_POWER)),// full blast
                new SleepAction(NON_INDEX_SPIN_TIME),
                new InstantAction(indexer::stopIndexerPower),
                new InstantAction(outtake::stop),
                new InstantAction(actuator::down),
                new InstantAction(() -> indexer.setIntaking(true)),
                new InstantAction(indexer::initializeColors),
                new InstantAction(() -> indexer.moveTo(Indexer.IndexerState.two)) // This means, if you don't move the indexer, the next intaken will enter slot 0
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

    public Action actionIntakeThree(Pose2d startActionPose, Pose2d startIntakePose, Pose2d endPose, MecanumDrive drive) {
        return drive.actionBuilder(startActionPose)
                .strafeToSplineHeading(startIntakePose.position, startIntakePose.heading)
                .afterTime(0, intake::run)
                .afterTime(ball1TimeDisp, () -> indexer.moveTo(indexer.getState().next()))
                .afterTime(ball2TimeDisp, () -> indexer.moveTo(indexer.getState().next()))
                .afterTime(timeToIntake, intake::stop)
                .strafeToLinearHeading(endPose.position, endPose.heading)
                .build();
    }

    public Action initializeAuto(Indexer.IndexerState startingSlot) { // only temporary for testing, this is done in actionQuickOuttake
        return new SequentialAction(
                new InstantAction(() -> indexer.initializeColors(Indexer.ArtifactColor.EMPTY)),
                new InstantAction(() -> indexer.setIntaking(true)),
                new InstantAction(() -> actuator.down()),
                new InstantAction(() -> indexer.moveTo(startingSlot))
        );
    }

    public Action initializeForIntake(Indexer.IndexerState slot) { // only temporary for testing, this is done in actionQuickOuttake

        return new SequentialAction(
            new InstantAction(() -> indexer.moveTo(slot))
        );
    }

    // apparantly its bad to do instant action and while loop but it works from testing
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
