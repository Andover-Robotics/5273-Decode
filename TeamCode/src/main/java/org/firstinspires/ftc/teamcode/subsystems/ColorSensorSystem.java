package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ColorSensorSystem {

    //Constants

    //evertyhign has to be a float?
    //Sensor gain to be tuned
    public static float SENSOR_GAIN = 20.0f;

    // Minimum alpha (light) required to consider the slot full
    public static float PRESENCE_ALPHA_THRESHOLD = 0.12f;

    // green range
    public static float GREEN_R_MIN = 0.00f;
    public static float GREEN_G_MIN = 0.4f;
    public static float GREEN_B_MIN = 0.3f;

    public static float GREEN_R_MAX = 0.35f;
    public static float GREEN_G_MAX = 1.00f;
    public static float GREEN_B_MAX = 1.0f;

    // purple range
    public static float PURPLE_R_MIN = 0.30f;
    public static float PURPLE_G_MIN = 0.40f;
    public static float PURPLE_B_MIN = 0.30f;

    public static float PURPLE_R_MAX = 0.7f;
    public static float PURPLE_G_MAX = 0.80f;
    public static float PURPLE_B_MAX = 1.00f;

    //hardware

    private final NormalizedColorSensor color;

    //constructor
    public ColorSensorSystem(HardwareMap hardwareMap) {
        color = hardwareMap.get(NormalizedColorSensor.class, "color");
        color.setGain(SENSOR_GAIN);
    }

    //internal helper funciotns
    private boolean inRange(float v, float min, float max) {
        return v >= min && v <= max;
    }

    private boolean matches(float r, float g, float b,
                            float rMin, float gMin, float bMin,
                            float rMax, float gMax, float bMax) {
        return inRange(r, rMin, rMax) &&
                inRange(g, gMin, gMax) &&
                inRange(b, bMin, bMax);
    }

    //api
    //insert that commetn about bruno mars from that other commit
    public boolean hasArtifact() {
        NormalizedRGBA rgba = color.getNormalizedColors();
        return rgba.alpha >= PRESENCE_ALPHA_THRESHOLD;
    }

    public Indexer.ArtifactColor classify() {
        NormalizedRGBA rgba = color.getNormalizedColors();

        if (rgba.alpha < PRESENCE_ALPHA_THRESHOLD) {
            return Indexer.ArtifactColor.EMPTY;
        }

        if (matches(rgba.red, rgba.green, rgba.blue,
                GREEN_R_MIN, GREEN_G_MIN, GREEN_B_MIN,
                GREEN_R_MAX, GREEN_G_MAX, GREEN_B_MAX)) {
            return Indexer.ArtifactColor.GREEN;
        }

        if (matches(rgba.red, rgba.green, rgba.blue,
                PURPLE_R_MIN, PURPLE_G_MIN, PURPLE_B_MIN,
                PURPLE_R_MAX, PURPLE_G_MAX, PURPLE_B_MAX)) {
            return Indexer.ArtifactColor.PURPLE;
        }

        return Indexer.ArtifactColor.UNKNOWN;
    }

    public Indexer.ArtifactColor classifyColorOnly() {
        NormalizedRGBA rgba = color.getNormalizedColors();

        if (matches(rgba.red, rgba.green, rgba.blue,
                GREEN_R_MIN, GREEN_G_MIN, GREEN_B_MIN,
                GREEN_R_MAX, GREEN_G_MAX, GREEN_B_MAX)) {
            return Indexer.ArtifactColor.GREEN;
        }

        if (matches(rgba.red, rgba.green, rgba.blue,
                PURPLE_R_MIN, PURPLE_G_MIN, PURPLE_B_MIN,
                PURPLE_R_MAX, PURPLE_G_MAX, PURPLE_B_MAX)) {
            return Indexer.ArtifactColor.PURPLE;
        }

        return Indexer.ArtifactColor.UNKNOWN;
    }

    //telem(tuff)

    public void addTelemetry(Telemetry telemetry) {
        NormalizedRGBA rgba = color.getNormalizedColors();

        telemetry.addLine("===== COLOR SENSOR =====");
        telemetry.addData("Detected Artifact", classify());
        telemetry.addData("R", "%.3f", rgba.red);
        telemetry.addData("G", "%.3f", rgba.green);
        telemetry.addData("B", "%.3f", rgba.blue);
        telemetry.addData("Alpha", "%.3f", rgba.alpha);
    }

    public float getAlpha() {
        return color.getNormalizedColors().alpha;
    }

    public float[] getRGB() {
        NormalizedRGBA rgba = color.getNormalizedColors();
        return new float[]{ rgba.red, rgba.green, rgba.blue };
    }
}
