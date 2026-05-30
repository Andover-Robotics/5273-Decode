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
import org.firstinspires.ftc.teamcode.subsystems.limelight.AprilTag;
import org.firstinspires.ftc.teamcode.subsystems.Aimer;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Outtake;

import java.util.function.IntSupplier;

@Config
public class BotActions {
    private final LinearOpMode opMode;
    private final Telemetry telemetry;
    private final Intake intake;
    private final Outtake outtake;
    private final Aimer aimer;
    public final AprilTag aprilTag;
    private final MecanumDrive drive;

    public static double NON_INDEX_SPIN_TIME = 2.5; //seconds of full-power indexer blast
    public static double FULL_BLAST_POWER = 0.25;

    public static double ball1TimeDisp = 0.66;
    public static double  ball2TimeDisp = 1.10;
    public static double  timeToIntake = 2.50;

    private boolean continuousLock = false;
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
        this.aprilTag = hardware.aprilTag;
        this.aimer = hardware.aimer;
        this.drive = hardware.mecanumDrive;
        this.telemetry = telemetry;
        this.opMode = opMode;
    }

    public Action actionStartOuttake(double rpm) {
        return new ParallelAction(
                new InstantAction(() -> outtake.set(rpm * quickspinRpmScale))
        );
    }

    public Action actionQuickOuttake() {
        return new SequentialAction(

        );
    }

    public Action actionSetSomeShizzle() {
        return new SequentialAction(

        );
    }

    public Action actionSetIntakeReverse() {
        return new InstantAction(intake::runBackwardsSlow);
    }

    public Action actionSetIntakePassive() {
        return new InstantAction(intake::runSlow);
    }

    public Action actionIntakeThree(Pose2d startActionPose, Pose2d startIntakePose, Pose2d endPose, MecanumDrive drive, double maxVel) {
        TranslationalVelConstraint velConstraint = new TranslationalVelConstraint(maxVel);

        return drive.actionBuilder(startActionPose)
                .strafeToSplineHeading(startIntakePose.position, startIntakePose.heading)
                .afterTime(0, intake::run)
                .afterTime(timeToIntake, intake::runSlow)
                .strafeToLinearHeading(endPose.position, endPose.heading, velConstraint)
                .build();
    }




    // bad to do instant action and while loop
    public Action actionScanObelisk() {
        return new Action() {
            private final ElapsedTime timer = new ElapsedTime();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                /*if (!opMode.opModeIsActive() || opMode.isStopRequested()) {
                    return false;
                }*/

                /*if (timer.seconds() > 8.0) {
                    telemetry.addLine("Obelisk scan timed out");
                    telemetry.update();
                    return false;
                }*/

                aprilTag.scanObeliskTag();
                obeliskId = aprilTag.getObeliskId();

                if (obeliskId == 21 || obeliskId == 22 || obeliskId == 23)
                    return false;

                return true;
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

                //telemetry.addData("obelisk id: ", obeliskId);
                //telemetry.update(); // could remove later

                if (continuousLock) {
                    aprilTag.scanGoalTag();
                    double bearing = aprilTag.getBearing();

                    double turn = 0;
                    if (!Double.isNaN(bearing)) {
                        turn = aimer.calculateTurnPowerFromBearing(bearing);
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

    public void setAprilTag(boolean trueFalse) {
        continuousLock = trueFalse;
    }

}