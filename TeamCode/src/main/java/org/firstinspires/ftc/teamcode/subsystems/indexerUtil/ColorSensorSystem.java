package org.firstinspires.ftc.teamcode.subsystems.indexerUtil;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

/**
 * ColorSensorSystem wraps the robot's normalized colour sensor and provides
 * methods to detect whether an artifact (game element) is present and, if so,
 * classify its colour as GREEN, PURPLE, EMPTY, or UNKNOWN.
 *
 * <p>Classification is performed in HSV (Hue, Saturation, Value) colour space
 * rather than raw RGB, because HSV separates the chromatic content (hue) from
 * the lighting level (value), making it more robust to ambient light variation.
 *
 * <p>Detection pipeline for a single sample:
 * <ol>
 *   <li>Read the normalized RGBA values from the sensor.</li>
 *   <li>If {@code alpha < PRESENCE_ALPHA_THRESHOLD} → EMPTY (nothing close enough).</li>
 *   <li>Convert RGB to HSV.</li>
 *   <li>If {@code saturation < MIN_SATURATION} or {@code value < MIN_VALUE} → UNKNOWN
 *       (artifact is too grey or too dark to determine colour reliably).</li>
 *   <li>If hue falls in the GREEN range → GREEN.</li>
 *   <li>If hue falls in the PURPLE range → PURPLE.</li>
 *   <li>Otherwise → UNKNOWN.</li>
 * </ol>
 *
 * <p>Annotated with {@link Config} so all thresholds and gain values can be
 * tuned live from the FTC Dashboard.
 */
@Config
public class ColorSensorSystem {

    // -----------------------------------------------------------------------
    // Sensor configuration
    // -----------------------------------------------------------------------

    /** Analog gain applied to the sensor's LED/photodiode pair.
     *  Higher gain increases sensitivity in dim environments but can saturate
     *  in bright light; value should be calibrated for the competition field. */
    public static float SENSOR_GAIN = 20.0f;

    // -----------------------------------------------------------------------
    // Presence detection threshold
    // -----------------------------------------------------------------------

    /** Minimum normalized alpha (light intensity) required to conclude that
     *  an artifact is physically close to the sensor.  Alpha is a proxy for
     *  how much light is reflected back; a value ≥ 0.8 indicates the sensor
     *  is very close to a surface (artifact present). */
    public static float PRESENCE_ALPHA_THRESHOLD = 0.8f;

    // -----------------------------------------------------------------------
    // Green hue range (degrees in HSV)
    // -----------------------------------------------------------------------

    /** Minimum hue angle (°) for the GREEN classification region. */
    public static float GREEN_H_MIN = 90f;

    /** Maximum hue angle (°) for the GREEN classification region. */
    public static float GREEN_H_MAX = 165f;

    // -----------------------------------------------------------------------
    // Purple hue range (degrees in HSV)
    // -----------------------------------------------------------------------

    /** Minimum hue angle (°) for the PURPLE classification region. */
    public static float PURPLE_H_MIN = 200f;

    /** Maximum hue angle (°) for the PURPLE classification region. */
    public static float PURPLE_H_MAX = 250f;

    // -----------------------------------------------------------------------
    // Quality gates (reject low-quality colour reads)
    // -----------------------------------------------------------------------

    /** Minimum saturation required for a colour read to be trusted.
     *  Very low saturation = grey-ish colour that does not map reliably to
     *  any specific hue range. */
    public static float MIN_SATURATION = 0.25f;

    /** Minimum value (brightness) required for a colour read to be trusted.
     *  Very dark readings produce unreliable hue values. */
    public static float MIN_VALUE = 0.10f;

    // -----------------------------------------------------------------------
    // Hardware
    // -----------------------------------------------------------------------

    /** The normalized colour sensor hardware object, mapped to the config
     *  name {@code "color"}. */
    private final NormalizedColorSensor color;

    // -----------------------------------------------------------------------
    // Constructor
    // -----------------------------------------------------------------------

    /**
     * Initialises the colour sensor system, retrieves the sensor from the
     * hardware map, and sets its analog gain.
     *
     * @param hardwareMap the robot's hardware map, used to look up the sensor
     *                    registered under the name {@code "color"}
     */
    public ColorSensorSystem(HardwareMap hardwareMap) {
        color = hardwareMap.get(NormalizedColorSensor.class, "color"); // fetch sensor by config name
        color.setGain(SENSOR_GAIN); // apply the configured sensor gain
    }

    // -----------------------------------------------------------------------
    // Public API
    // -----------------------------------------------------------------------

    /**
     * Returns {@code true} if the sensor detects that an artifact is present
     * (i.e., {@code alpha >= PRESENCE_ALPHA_THRESHOLD}).
     *
     * <p>Alpha is used as a proximity indicator because a highly reflective
     * surface at close range returns much more light than an empty slot.
     *
     * @return {@code true} if an artifact is close to the sensor
     */
    public boolean hasArtifact() {
        NormalizedRGBA rgba = color.getNormalizedColors(); // single sensor read
        return rgba.alpha >= PRESENCE_ALPHA_THRESHOLD;     // presence check by reflected light level
    }

    /**
     * Classifies the artifact colour (or EMPTY) based on a single sensor sample.
     * Returns EMPTY if no artifact is detected (alpha below threshold);
     * otherwise delegates to the HSV classifier.
     *
     * @return classified {@link Indexer.ArtifactColor}
     */
    public Indexer.ArtifactColor classify() {
        NormalizedRGBA rgba = color.getNormalizedColors(); // read RGBA from hardware
        if (rgba.alpha < PRESENCE_ALPHA_THRESHOLD) {
            return Indexer.ArtifactColor.EMPTY; // not close enough to an artifact; slot is empty
        }
        return classifyColorOnlyFromRGBA(rgba); // presence confirmed: classify the colour
    }

    /**
     * Classifies the colour assuming an artifact <em>is</em> present.
     * Ignores the alpha/presence check and only evaluates the chromatic content.
     * Use this when the caller has already confirmed presence by other means.
     *
     * @return classified {@link Indexer.ArtifactColor} (UNKNOWN if colour is ambiguous)
     */
    public Indexer.ArtifactColor classifyColorOnly() {
        NormalizedRGBA rgba = color.getNormalizedColors(); // read RGBA from hardware
        return classifyColorOnlyFromRGBA(rgba);            // classify without checking alpha
    }

    /**
     * Adds a detailed colour sensor readout to the telemetry display.
     * Includes raw RGBA values, converted HSV values, detected artifact colour,
     * and the presence flag.  Used during tuning sessions.
     *
     * @param telemetry the FTC telemetry object to write data to
     */
    public void addTelemetry(Telemetry telemetry) {
        NormalizedRGBA rgba = color.getNormalizedColors();   // read RGBA once
        float[] hsv = rgbToHsv(rgba.red, rgba.green, rgba.blue); // convert to HSV for display

        telemetry.addLine("===== COLOR SENSOR (HSV) =====");
        telemetry.addData("Detected Artifact", classify());       // full classification result
        telemetry.addData("Has Artifact",       hasArtifact());   // presence flag

        // Raw RGBA components (normalized 0–1 by the SDK)
        telemetry.addData("R", "%.4f", rgba.red);
        telemetry.addData("G", "%.4f", rgba.green);
        telemetry.addData("B", "%.4f", rgba.blue);
        telemetry.addData("Alpha", "%.4f", rgba.alpha);

        // HSV components derived from the RGB reading
        telemetry.addData("H", "%.1f°", hsv[0]); // hue in degrees [0, 360)
        telemetry.addData("S", "%.4f",  hsv[1]); // saturation [0, 1]
        telemetry.addData("V", "%.4f",  hsv[2]); // value/brightness [0, 1]
    }

    /**
     * Returns the raw normalized alpha (light intensity) from the sensor.
     * Values close to 1.0 indicate a very nearby surface; values near 0
     * indicate an empty/far slot.
     *
     * @return normalized alpha in the range [0, 1]
     */
    public float getAlpha() {
        return color.getNormalizedColors().alpha; // single read, return alpha only
    }

    /**
     * Returns the raw normalized RGB values as a three-element float array.
     *
     * @return {@code float[]{ red, green, blue }} each in [0, 1]
     */
    public float[] getRGB() {
        NormalizedRGBA rgba = color.getNormalizedColors();
        return new float[]{ rgba.red, rgba.green, rgba.blue };
    }

    /**
     * Returns the HSV representation of the current sensor reading.
     *
     * @return {@code float[]{ hue°, saturation, value }} where hue ∈ [0, 360),
     *         saturation ∈ [0, 1], value ∈ [0, 1]
     */
    // its so peak
    public float[] getHSV() {
        NormalizedRGBA rgba = color.getNormalizedColors();           // read RGB
        return rgbToHsv(rgba.red, rgba.green, rgba.blue);            // convert and return
    }

    // -----------------------------------------------------------------------
    // Internal classification helpers
    // -----------------------------------------------------------------------

    /**
     * Internal method: classifies colour from an already-read RGBA sample.
     * Assumes presence has already been confirmed (alpha check done externally).
     *
     * <p>Steps:
     * <ol>
     *   <li>Convert RGB to HSV.</li>
     *   <li>If saturation or value are below quality thresholds → UNKNOWN.</li>
     *   <li>Check GREEN hue range → GREEN.</li>
     *   <li>Check PURPLE hue range → PURPLE.</li>
     *   <li>Default → UNKNOWN.</li>
     * </ol>
     *
     * @param rgba the RGBA sample to classify
     * @return classified {@link Indexer.ArtifactColor}
     */
    private Indexer.ArtifactColor classifyColorOnlyFromRGBA(NormalizedRGBA rgba) {
        float[] hsv = rgbToHsv(rgba.red, rgba.green, rgba.blue); // convert to HSV
        float h = hsv[0]; // hue in degrees
        float s = hsv[1]; // saturation [0, 1]
        float v = hsv[2]; // value/brightness [0, 1]

        // Quality gate: reject achromatic or very dark readings.
        if (s < MIN_SATURATION || v < MIN_VALUE) {
            return Indexer.ArtifactColor.UNKNOWN; // colour is too grey or too dark to trust
        }

        // Check whether the hue falls in the green range.
        if (inHueRange(h, GREEN_H_MIN, GREEN_H_MAX)) {
            return Indexer.ArtifactColor.GREEN;
        }

        // Check whether the hue falls in the purple range.
        if (inHueRange(h, PURPLE_H_MIN, PURPLE_H_MAX)) {
            return Indexer.ArtifactColor.PURPLE;
        }

        return Indexer.ArtifactColor.UNKNOWN; // hue did not match any known colour range
    }

    /**
     * Returns {@code true} if {@code hueDeg} falls within the circular hue
     * range [minDeg, maxDeg].  Handles wrap-around ranges that cross 0°/360°
     * (e.g., red hues near 0°/360°).
     *
     * @param hueDeg hue angle in degrees to test
     * @param minDeg start of the hue range (inclusive)
     * @param maxDeg end of the hue range (inclusive)
     * @return {@code true} if the hue is within the specified range
     */
    private boolean inHueRange(float hueDeg, float minDeg, float maxDeg) {
        hueDeg = wrapHue(hueDeg); // normalize input to [0, 360)
        minDeg = wrapHue(minDeg);
        maxDeg = wrapHue(maxDeg);

        if (minDeg <= maxDeg) {
            // Non-wrapping range: simply check if hue is between min and max.
            return hueDeg >= minDeg && hueDeg <= maxDeg;
        } else {
            // Wrap-around range (e.g., 300–30°): hue is in range if ≥ min OR ≤ max.
            return hueDeg >= minDeg || hueDeg <= maxDeg;
        }
    }

    /**
     * Wraps a hue value into [0, 360).
     *
     * @param h hue angle in degrees (any value)
     * @return equivalent hue in [0, 360)
     */
    private float wrapHue(float h) {
        h %= 360f;           // reduce to (-360, 360)
        if (h < 0) h += 360f; // shift negatives into [0, 360)
        return h;
    }

    /**
     * Converts normalized RGB to HSV colour space.
     *
     * <p>The conversion follows the standard algorithm:
     * <ul>
     *   <li>V = max(R, G, B)</li>
     *   <li>S = (max − min) / max  (or 0 if max ≈ 0)</li>
     *   <li>H depends on which channel is maximum; varies in [0°, 360°)</li>
     * </ul>
     *
     * @param r normalized red channel [0, 1]
     * @param g normalized green channel [0, 1]
     * @param b normalized blue channel [0, 1]
     * @return {@code float[]{ H, S, V }} where H ∈ [0, 360), S ∈ [0, 1], V ∈ [0, 1]
     */
    // Normalized RGB to HSV conversion
    private float[] rgbToHsv(float r, float g, float b) {
        float max   = Math.max(r, Math.max(g, b)); // largest channel = Value
        float min   = Math.min(r, Math.min(g, b)); // smallest channel
        float delta = max - min;                    // chroma = range of the three channels

        float h;
        if (delta < 1e-6f) {
            // All channels equal → achromatic grey; hue is undefined, use 0.
            h = 0f;
        } else if (max == r) {
            // Red is dominant channel: hue in [−60°, 60°) relative to red.
            h = 60f * (((g - b) / delta) % 6f);
        } else if (max == g) {
            // Green is dominant: hue in [60°, 180°).
            h = 60f * (((b - r) / delta) + 2f);
        } else {
            // Blue is dominant: hue in [180°, 300°).
            h = 60f * (((r - g) / delta) + 4f);
        }
        if (h < 0f) h += 360f; // ensure hue is non-negative

        float s = (max <= 1e-6f) ? 0f : (delta / max); // saturation; 0 for near-black
        float v = max;                                   // value = brightness = max channel

        return new float[]{ h, s, v };
    }
}