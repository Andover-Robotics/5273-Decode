package org.firstinspires.ftc.teamcode.subsystems.limelight;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * AprilTag wraps the Limelight 3A vision co-processor to detect and measure
 * fiducial AprilTag markers on the FTC field.
 *
 * <p>Two types of AprilTag scans are supported:
 * <ul>
 *   <li><b>Goal tag scan</b> ({@link #scanGoalTag()}) – reads the alliance
 *       scoring goal tag using pipeline 0 (blue) or 1 (red).  Populates
 *       {@link #bearing}, {@link #elevation}, and {@link #range} for aiming.</li>
 *   <li><b>Obelisk tag scan</b> ({@link #scanObeliskTag()}) – reads the centre
 *       obelisk tag (pipeline 2) to determine the current scoring motif pattern
 *       (IDs 21, 22, 23).  The result is cached in the static field
 *       {@link #obeliskId} so all class instances share it.</li>
 * </ul>
 *
 * <p>Range is computed from the tag's elevation angle using a geometric formula
 * based on the camera height and the known tag height above the floor.
 *
 * <p><b>Alliance-specific notes:</b>
 * <ul>
 *   <li>Blue alliance: use {@link #setPipeline(int) setPipeline(0)} and
 *       {@code setGoalTagID(20)} (example; actual IDs TBD).</li>
 *   <li>Red alliance: use {@link #setPipeline(int) setPipeline(1)} and
 *       {@code setGoalTagID(24)}.</li>
 * </ul>
 */
// TODO IMPORTANT NOTES: For goalTagID, just have separate teleops one for red alliance one for blue where blue teleop can setGoalTagID(20) and red teleop can setGoalTagID(24)
// TODO We will see whether we want separate auto for either alliance, probably yes its just easier that way and there may be some functionality requiring that.
public class AprilTag {

    // -----------------------------------------------------------------------
    // Scan results (updated by scan methods)
    // -----------------------------------------------------------------------

    /** ID of the last tag seen by {@link #scanGoalTag()}.
     *  {@code -1} if no goal tag has been detected yet. */
    private int id = -1;

    /** ID of the last obelisk tag seen by {@link #scanObeliskTag()}.
     *  Shared across all instances (static) so it can be read globally.
     *  {@code -1} if no obelisk tag has been detected. */
    public static int obeliskId = -1;

    /** ID of the target alliance goal tag.  Set externally before scanning
     *  so the scan methods can filter for the correct tag. */
    private int goalTagID; // our current alliance goal

    /** The most recently detected goal tag ID.  May differ from {@link #goalTagID}
     *  if the camera sees a tag other than the intended goal. */
    private int cameraScannedId;

    /** Horizontal bearing to the goal tag in degrees (positive = tag is to the
     *  right of the camera center; negative = to the left). */
    private double bearing;

    /** Vertical elevation angle of the goal tag in degrees above the camera
     *  horizontal (positive = tag is above camera center). */
    private double elevation;

    /** Straight-line distance from the camera to the goal tag in inches,
     *  computed from the elevation angle and known heights. */
    private double range;

    /** Pixel area fraction of the goal tag in the camera frame.
     *  Larger = closer; used as a rough confidence metric. */
    private double tagSize;

    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------

    /** The Limelight 3A co-processor hardware object, mapped to config name
     *  {@code "limelight"}. */
    private final Limelight3A limelight;

    /** FTC telemetry reference (currently unused but reserved for debug output). */
    private final Telemetry telemetry;

    // -----------------------------------------------------------------------
    // Camera / target geometry constants
    // -----------------------------------------------------------------------

    /** Height of the Limelight camera above the floor in inches. */
    private final double LIMELIGHT_HEIGHT = 11.815; // inches

    /** Upward tilt angle of the Limelight camera from horizontal in degrees.
     *  Added to the tag's reported elevation when computing the range. */
    private final double LIMELIGHT_ANGLE = 15; // degrees above horizontal

    /** Height of the goal AprilTag above the floor in inches. */
    private final double TARGET_HEIGHT = 29.5; // inches

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Creates an AprilTag instance, retrieves the Limelight from the hardware
     * map, switches to pipeline 0 (blue goal), and starts the vision pipeline.
     *
     * @param hardwareMap the robot's hardware map used to find the Limelight
     *                    device registered under the config name {@code "limelight"}
     * @param telemetry   the FTC telemetry object (reserved for future debug output)
     */
    public AprilTag(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        // Retrieve the Limelight co-processor from the hardware map.
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0); // default to pipeline 0 (blue alliance goal)
        limelight.start();           // begin streaming and processing frames
    }

    // -----------------------------------------------------------------------
    // Limelight power control
    // -----------------------------------------------------------------------

    /**
     * Starts or stops the Limelight pipeline.
     * Stopping it when not in use conserves battery and reduces CPU load.
     *
     * @param bool {@code true} to start the pipeline; {@code false} to stop it
     */
    public void toggle(boolean bool) {
        if (bool) { limelight.start(); } // resume vision processing
        else      { limelight.stop();  } // pause vision processing
    }

    // -----------------------------------------------------------------------
    // Scan methods
    // -----------------------------------------------------------------------

    /**
     * Scans for the obelisk AprilTag (pipeline 2) and stores the detected
     * tag ID in {@link #obeliskId}.
     *
     * <p>The obelisk tag ID encodes the scoring motif pattern for the current
     * round (IDs 21, 22, or 23).  Reads the first fiducial in the result list
     * and returns immediately; only one obelisk tag is expected to be visible.
     */
    public void scanObeliskTag() {
        setPipeline(2); // switch to pipeline 2 which is configured for obelisk tags

        // Get all fiducial detections from the latest frame.
        List<LLResultTypes.FiducialResult> scanned = limelight
                .getLatestResult()
                .getFiducialResults();

        // Store the first detected tag ID; only one obelisk tag is expected.
        for (LLResultTypes.FiducialResult detection : scanned) {
            int fid = detection.getFiducialId(); // read the numeric tag ID
            obeliskId = fid;                     // cache globally for other classes to read
            return;                              // only need one detection; stop after the first
        }
    }

    // -----------------------------------------------------------------------
    // Internal geometry helpers
    // -----------------------------------------------------------------------

    /**
     * Computes the slant range (inches) from the camera to the tag using the
     * known vertical geometry.
     *
     * <p>Formula:
     * <pre>
     *   range = (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / sin(elevation + LIMELIGHT_ANGLE)
     * </pre>
     * where all angles are in degrees.
     *
     * @param elevation vertical angle of the tag above camera centre (degrees)
     * @return slant distance from camera to tag in inches
     */
    private double calculateDistance(double elevation) {
        // sin() of the total angle from horizontal to the tag gives the vertical component;
        // dividing the known height difference yields the slant distance.
        return (TARGET_HEIGHT - LIMELIGHT_HEIGHT) / Math.sin(Math.toRadians(elevation + LIMELIGHT_ANGLE));
    }

    // -----------------------------------------------------------------------
    // Goal tag scan
    // -----------------------------------------------------------------------

    /**
     * Scans for the alliance goal AprilTag and updates {@link #bearing},
     * {@link #elevation}, {@link #range}, and {@link #tagSize}.
     *
     * <p>If no tag is found in the current frame, {@link #bearing},
     * {@link #elevation}, and {@link #range} are set to {@link Double#NaN}
     * so callers can detect a "tag lost" condition.  {@link #id} is reset to
     * {@code -1}.
     *
     * <p>If multiple tags are visible the last one in the list "wins" (all are
     * iterated); in practice only one goal tag should be in frame.
     */
    public void scanGoalTag() {
        id = -1;                     // reset to no-detection state
        /* So that if you scan and there's no tag, range stays NaN (bearing should be reset in loops) */
        bearing   = Double.NaN;      // no valid bearing until a tag is found
        elevation = Double.NaN;      // no valid elevation until a tag is found
        range     = Double.NaN;      // no valid range until a tag is found

        // Get all fiducial detections from the latest frame.
        // If camera is facing to the right of center the bearing is positive.
        List<LLResultTypes.FiducialResult> scanned = limelight.getLatestResult().getFiducialResults();
        for (LLResultTypes.FiducialResult detection: scanned) {
            cameraScannedId = detection.getFiducialId();       // record which tag was seen
            // goalTagID should be obtained before the round / during auto
            id        = cameraScannedId;                       // store as "current" goal tag ID
            elevation = detection.getTargetYDegrees();         // vertical angle from camera centre (degrees)
            range     = calculateDistance(elevation);          // compute slant range from elevation
            bearing   = detection.getTargetXDegrees();        // horizontal angle from camera centre (degrees)
            tagSize   = detection.getTargetArea();             // fractional pixel area of the tag
        }
    }

    // -----------------------------------------------------------------------
    // Configuration
    // -----------------------------------------------------------------------

    /**
     * Switches the active Limelight pipeline.
     *
     * <p>Pipeline assignments:
     * <ul>
     *   <li>0 – Blue alliance goal tag</li>
     *   <li>1 – Red alliance goal tag</li>
     *   <li>2 – Obelisk tag (motif pattern)</li>
     * </ul>
     *
     * @param pipeline pipeline index to activate (0, 1, or 2)
     */
    public void setPipeline(int pipeline) {
        // 0 blue, 1 red, 2 obelisk
        limelight.pipelineSwitch(pipeline); // send the pipeline switch command to the co-processor
    }

    // -----------------------------------------------------------------------
    // Getters
    // -----------------------------------------------------------------------

    /**
     * Returns the ID of the most recently detected goal tag.
     *
     * @return AprilTag ID, or 0 if no tag has been scanned yet
     */
    public int getCurrentId() {
        return cameraScannedId; // last goal tag ID seen by the camera
    }

    /**
     * Returns the ID of the most recently detected obelisk tag.
     * This is a static field shared across all instances.
     *
     * @return obelisk AprilTag ID (21, 22, or 23 for valid motif tags),
     *         or {@code -1} if none detected
     */
    public int getObeliskId() {
        return obeliskId; // shared static field; same value regardless of which instance reads it
    }

    /**
     * Returns the vertical elevation of the goal tag in degrees above the
     * camera's horizontal axis.
     *
     * @return elevation angle in degrees; {@link Double#NaN} if no tag detected
     */
    public double getElevation() {
        return elevation; // degrees above camera centre; used to compute range
    }

    /**
     * Returns the computed slant range to the goal tag in inches.
     *
     * @return range in inches; {@link Double#NaN} if no tag detected
     */
    public double getRange() {
        return range; // slant distance from camera to goal tag (inches)
    }

    /**
     * Returns the horizontal bearing to the goal tag in degrees.
     * Positive = tag is to the right of the camera; negative = to the left.
     *
     * @return bearing in degrees; {@link Double#NaN} if no tag detected
     */
    public double getBearing() {
        return bearing; // horizontal angle from camera centre to goal tag (degrees)
    }

    /**
     * Returns the fractional pixel area of the last detected goal tag.
     * A larger value indicates the robot is closer to the tag.
     *
     * @return tag area fraction (0.0–100.0); 0 if no tag detected
     */
    public double getArea() {
        return tagSize; // tag pixel area as a fraction of the frame; proxy for proximity
    }

    /**
     * Manually overrides the cached camera-scanned goal tag ID.
     * Useful during testing or when the obelisk tag ID is known in advance.
     *
     * @param i the tag ID to store as the current camera-scanned ID
     */
    public void setCurrentCameraScannedId(int i) {
        cameraScannedId = i; // override the last detected goal tag ID
    }
}