package org.firstinspires.ftc.teamcode.subsystems.indexerUtil;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Indexer;

@Config
public class ColorSensorSystem {

    // Sensor gain to be tuned
    public static float SENSOR_GAIN = 20.0f;

    // Minimum alpha (light) required to consider the slot full
    public static float PRESENCE_ALPHA_THRESHOLD = 0.40f;

    // HSV classification
    // Hue is in degrees
    public static float GREEN_H_MIN = 90f;
    public static float GREEN_H_MAX = 165f;

    public static float PURPLE_H_MIN = 200f;
    public static float PURPLE_H_MAX = 250f;

    // Gate out low quality color (gray/too dark)
    public static float MIN_SATURATION = 0.25f;
    public static float MIN_VALUE = 0.10f;

    // hardware
    private final NormalizedColorSensor color;

    // constructor
    public ColorSensorSystem(HardwareMap hardwareMap) {
        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        color.setGain(SENSOR_GAIN);
    }

    // api
    public boolean hasArtifact() {
        NormalizedRGBA rgba = color.getNormalizedColors();
        return rgba.alpha >= PRESENCE_ALPHA_THRESHOLD;
    }

    public Indexer.ArtifactColor classify() {
        NormalizedRGBA rgba = color.getNormalizedColors();

        if (rgba.alpha < PRESENCE_ALPHA_THRESHOLD) {
            return Indexer.ArtifactColor.EMPTY;
        }

        return classifyColorOnlyFromRGBA(rgba);
    }

    public Indexer.ArtifactColor classifyColorOnly() {
        NormalizedRGBA rgba = color.getNormalizedColors();
        return classifyColorOnlyFromRGBA(rgba);
    }

    public void addTelemetry(Telemetry telemetry) {
        NormalizedRGBA rgba = color.getNormalizedColors();
        float[] hsv = rgbToHsv(rgba.red, rgba.green, rgba.blue);

        telemetry.addLine("===== COLOR SENSOR (HSV) =====");
        telemetry.addData("Detected Artifact", classify());
        telemetry.addData("Has Artifact", hasArtifact());

        telemetry.addData("R", "%.4f", rgba.red);
        telemetry.addData("G", "%.4f", rgba.green);
        telemetry.addData("B", "%.4f", rgba.blue);
        telemetry.addData("Alpha", "%.4f", rgba.alpha);

        telemetry.addData("H", "%.1f°", hsv[0]);
        telemetry.addData("S", "%.4f", hsv[1]);
        telemetry.addData("V", "%.4f", hsv[2]);
    }

    public float getAlpha() {
        return color.getNormalizedColors().alpha;
    }

    public float[] getRGB() {
        NormalizedRGBA rgba = color.getNormalizedColors();
        return new float[]{ rgba.red, rgba.green, rgba.blue };
    }

    //its so peak
    public float[] getHSV() {
        NormalizedRGBA rgba = color.getNormalizedColors();
        return rgbToHsv(rgba.red, rgba.green, rgba.blue);
    }

    //helprs

    private Indexer.ArtifactColor classifyColorOnlyFromRGBA(NormalizedRGBA rgba) {
        float[] hsv = rgbToHsv(rgba.red, rgba.green, rgba.blue);
        float h = hsv[0];
        float s = hsv[1];
        float v = hsv[2];

        if (s < MIN_SATURATION || v < MIN_VALUE) {
            return Indexer.ArtifactColor.UNKNOWN;
        }

        if (inHueRange(h, GREEN_H_MIN, GREEN_H_MAX)) {
            return Indexer.ArtifactColor.GREEN;
        }

        if (inHueRange(h, PURPLE_H_MIN, PURPLE_H_MAX)) {
            return Indexer.ArtifactColor.PURPLE;
        }

        return Indexer.ArtifactColor.UNKNOWN;
    }

    //
    private boolean inHueRange(float hueDeg, float minDeg, float maxDeg) {
        hueDeg = wrapHue(hueDeg);
        minDeg = wrapHue(minDeg);
        maxDeg = wrapHue(maxDeg);

        if (minDeg <= maxDeg) {
            return hueDeg >= minDeg && hueDeg <= maxDeg;
        } else {
            // wrap-around (e.g., 300..30)
            return hueDeg >= minDeg || hueDeg <= maxDeg;
        }
    }

    private float wrapHue(float h) {
        h %= 360f;
        if (h < 0) h += 360f;
        return h;
    }

    //Normalised RGB to HSV conversion
    private float[] rgbToHsv(float r, float g, float b) {
        float max = Math.max(r, Math.max(g, b));
        float min = Math.min(r, Math.min(g, b));
        float delta = max - min;

        float h;
        if (delta < 1e-6f) {
            h = 0f;
        } else if (max == r) {
            h = 60f * (((g - b) / delta) % 6f);
        } else if (max == g) {
            h = 60f * (((b - r) / delta) + 2f);
        } else {
            h = 60f * (((r - g) / delta) + 4f);
        }
        if (h < 0f) h += 360f;

        float s = (max <= 1e-6f) ? 0f : (delta / max);
        float v = max;

        return new float[]{ h, s, v };
    }
}