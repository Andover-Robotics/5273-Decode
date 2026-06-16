package org.firstinspires.ftc.teamcode.auto.utils;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.auto.roadrunner.miscRR.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Storage;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.Aimer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

@Config
public class BotActions {
    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    private final Intake intake;
    private final Storage storage;
    private final Outtake outtake;
    private final Turret turret;
    private final Aimer aimer;
    public final AprilTag aprilTag;
    private final MecanumDrive drive;

    public static double NON_INDEX_SPIN_TIME = 2.5; //seconds of full-power indexer blast
    public static double FULL_BLAST_POWER = 0.25;

    public static double ball1TimeDisp = 0.66;
    public static double  ball2TimeDisp = 1.10;
    public static double  timeToIntake = 2.50;

    private boolean aimlock = false;
    private long lastAimUpdate = 0;
    private static final long AIM_UPDATE_INTERVAL_MS = 0;
    public static double FIRE_TIME = 0.85;
    protected double[] targetData = {0,0,0};
    public static double withinRpmRange = 150;
    public static double targetRPM = 0;
    private double turnCorrection = 0.0;
    private double bearingTurnCorrection = 0.0;


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
        this.outtake = hardware.outtake;
        this.turret = hardware.turret;
        this.aprilTag = hardware.aprilTag;
        this.aimer = hardware.aimer;
        this.drive = hardware.mecanumDrive;
        this.storage = hardware.storage;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    public Action startIntake() {
        return new ParallelAction(
                new InstantAction(intake::run),
                new InstantAction(storage::runTransfer)
        );
    }

    public Action runContinuousIntake() {
        return new ParallelAction(
                new InstantAction(intake::runSlow),
                new InstantAction(storage::stopTransfer)
        );
    }


    public Action startOuttake() {
        return new InstantAction(() -> outtake.set(getTargetRPM() * quickspinRpmScale));
    }

    public Action stopOuttake() {
        return new InstantAction(() -> outtake.stop());
    }

    public Action actionSetAimlock(boolean aimlock) {
        return new InstantAction(() -> setAimlock(aimlock));
    }

    public Action actionOuttake() {
        Action shootingAction = new SequentialAction(
                packet -> {
                    outtake.set(getTargetRPM());
                    return !outtake.inRange(withinRpmRange);
                },
                new InstantAction(intake::run),
                new InstantAction(storage::runTransfer),
                new InstantAction(storage::openGate),
                new SleepAction(FIRE_TIME),
                new InstantAction(storage::closeGate),
                new InstantAction(intake::stop),
                new InstantAction(storage::stopTransfer)
        );

        return new ParallelAction(
                shootingAction,
                packet -> {
                    // Packet returns true when shootingAction ends
                    outtake.set(getTargetRPM());
                    return shootingAction.run(packet);
                }
        );
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

                //telemetry.addData("obelisk id: ", obeliskId);
                //telemetry.update(); // could remove later

                targetData = aimer.calculateLocalizedData();
                lastTurnCorrection = targetData[0];
                targetRPM = outtake.getRegressionRPM(targetData[1]);
                bearingTurnCorrection = targetData[2];
                turnCorrection = lastTurnCorrection;

                if (aimlock)
                    turret.rotate(bearingTurnCorrection);

                return true;
            }
        };
    }

    public Action startActions(Aimer.Goal goal) {
        if (goal == Aimer.Goal.BLUE) {
            return new SequentialAction (
                    new InstantAction(aimer::setBlueTarget),
                    new InstantAction(storage::closeGate),
                    new InstantAction(turret::initialize),
                    new InstantAction(intake::runSlow),
                    new InstantAction(() -> setAimlock(true))
            );
        }
        else {
            return new SequentialAction(
                    new InstantAction(aimer::setRedTarget),
                    new InstantAction(storage::closeGate),
                    new InstantAction(turret::initialize),
                    new InstantAction(intake::runSlow),
                    new InstantAction(() -> setAimlock(true))
            );
        }
    }

    public double getTargetRPM() {
        return targetRPM;
    }

    public void setAimlock(boolean aimlock) {
        this.aimlock = aimlock;
    }

    public int getObeliskId() {
        return obeliskId;
    }

}