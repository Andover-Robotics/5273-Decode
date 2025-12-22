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

    private final double LIMELIGHT_HEIGHT = 11.5;
    private final double LIMELIGHT_ANGLE = 15;
    private final double TARGET_HEIGHT = 29.5;
    private final double LIMELIGHT_TO_CENTER = 4;

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

    private double getBotAngle(double cameraAngle) { // cameraAngle = angle from limelight looking forward line to line that goes from limelight to april tag
        double a = range; // range from limelight to april tag
        double b = LIMELIGHT_TO_CENTER; // distance from limelight to center front of robot
        double x = Math.toRadians(90 - cameraAngle); // angle between line that goes from limelight to apriltag and the front of the robot
        double c = Math.sqrt(a * a + b * b - 2 * a * b * Math.cos(x)); // range from center front of robot to apriltag
        double y = Math.asin(Math.sin(x) * a / c); // angle between line that goes from center front of robot to apriltag and front of the robot
        if (b * b + c * c < a * a) y = Math.PI - y;
        telemetry.addData("cameraAngle",cameraAngle);
        telemetry.addData("a",a);
        telemetry.addData("b",b);
        telemetry.addData("x",x);
        telemetry.addData("c",c);
        telemetry.addData("y",y);
        return Math.toDegrees(y) - 90; // angle from center front of robot to april tag
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
                tagSize = detection.getTargetArea();
                bearing = getBotAngle(detection.getTargetXDegrees());
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