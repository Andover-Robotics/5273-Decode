package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

// TODO IMPORTANT NOTES: For goalTagID, just have separate teleops one for red alliance one for blue where blue teleop can setGoalTagID(20) and red teleop can setGoalTagID(24)
// TODO We will see whether we want separate auto for either alliance, probably yes its just easier that way and there may be some functionality requiring that.
public class AprilTag {
    private int id;
    private int obeliskId;
    private int goalTagID; // our current alliance goal
    private int cameraScannedId;
    private double bearing;
    private double elevation;
    private double range;
    private double tagSize;
    private final Limelight3A limelight;
    private final Telemetry telemetry;

    private final double LIMELIGHT_HEIGHT = 11.815;
    private final double LIMELIGHT_ANGLE = 15;
    private final double TARGET_HEIGHT = 29.5;

    public AprilTag(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void toggle(boolean bool) {
        if (bool) { limelight.start(); }
        else { limelight.stop(); }
    }

    public void scanObeliskTag() {
        id = -1;
        List<LLResultTypes.FiducialResult> scanned = limelight.getLatestResult().getFiducialResults();

        for (LLResultTypes.FiducialResult detection: scanned) {
            int id = detection.getFiducialId();
            if (id >= 21 && id <= 23) {
                obeliskId = id;
            }
        }
    }

    private double calculateDistance(double elevation) {
        return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.sin(Math.toRadians(elevation + LIMELIGHT_ANGLE));
    }

    public void scanGoalTag() {
        id = -1;
        bearing = Double.NaN;
        elevation = Double.NaN;
        range = Double.NaN;

        // If camera is facing to the right of the center of the cam (if it needs to move to the left) the bearing is positive.
        List<LLResultTypes.FiducialResult> scanned = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult detection: scanned) {
            cameraScannedId = detection.getFiducialId();
            // goalTagID should be gotten before round/during auto
            if (cameraScannedId == goalTagID) {
                id = cameraScannedId;
                elevation = detection.getTargetYDegrees();
                range = calculateDistance(elevation);
                bearing = detection.getTargetXDegrees();
                tagSize = detection.getTargetArea();
                break;
            }
        }
    }

    public void setGoalTagID(int allianceTagID) {
        goalTagID = allianceTagID;
    }
    public int getCurrentId() {
        return cameraScannedId;
    }
    public int getObeliskId() {
        return obeliskId;
    }
    public double getElevation() {
        return elevation;
    }
    public double getRange() {
        return range;
    }
    public double getBearing() {
        return bearing;
    }
    public double getArea() {
        return tagSize;
    }
    public void setCurrentCameraScannedId(int i) {
        cameraScannedId = i;
    }
}