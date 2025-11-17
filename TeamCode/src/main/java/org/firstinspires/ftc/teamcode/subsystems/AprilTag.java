package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
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
    private final VisionPortal portal;
    private final AprilTagProcessor processor;

    public AprilTag(HardwareMap hardwareMap) {
        AprilTagLibrary library = AprilTagGameDatabase.getCurrentGameTagLibrary();

        processor = new AprilTagProcessor.Builder()
                .setTagLibrary(library)
                .build();

        WebcamName webcamname = hardwareMap.get(WebcamName.class, "webcam");
        portal = VisionPortal.easyCreateWithDefaults(webcamname, processor);
    }

    public void toggle(boolean bool) {
        portal.setProcessorEnabled(processor, bool);
    }

    public void scanObeliskTag() {
        id = -1;
        List<AprilTagDetection> detectionList = processor.getDetections();

        for (AprilTagDetection detection : detectionList) {
            if (detection.id >= 21 && detection.id <= 23) {
                obeliskId = detection.id;
            }
        }
    }

    public void scanGoalTag() {
        id = -1;
        bearing = Double.NaN;
        elevation = Double.NaN;
        range = Double.NaN;

        // If camera is facing to the right of the center of the cam (if it needs to move to the left) the bearing is positive.
        List<AprilTagDetection> detectionList = processor.getDetections();
        for (AprilTagDetection detection : detectionList) {
            cameraScannedId = detection.id;
            // goalTagID should be gotten before round/during auto
            if (detection.id == goalTagID && detection.ftcPose != null) {
                id = detection.id;
                bearing = detection.ftcPose.bearing;
                elevation = detection.ftcPose.elevation;
                range = detection.ftcPose.range;
                break;
            }
        }
    }

    public void setGoalTagID(int allianceTagID) {
        goalTagID = allianceTagID;
    }
    public int getCurrentId() { return cameraScannedId;}
    public int getObeliskId(){
        return obeliskId;
    }
    public double getElevation(){
        return elevation;
    }
    public double getRange(){
        return range;
    }
    public double getBearing(){
        return bearing;
    }
    public void setCurrentCameraScannedId(int i) {
        cameraScannedId = i;
    }
}
