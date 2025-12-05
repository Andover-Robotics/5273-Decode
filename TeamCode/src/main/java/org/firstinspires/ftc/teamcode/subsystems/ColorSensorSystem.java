package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ColorSensorSystem {

    private final int[] GREEN_RGB = {0,255,0};
    private final double GREEN_TOLERANCE = 0.5; //50%
    private final int[] PURPLE_RGB = {128,0,128};
    private final double PURPLE_TOLERANCE = 0.5; //50%
    private final ColorSensor colorSensor;

    public ColorSensorSystem(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "color");
    }

    // checks if the int provided is less than a max and a min calculated using a starting int and a tolerance percent
    private boolean checkColorRange(int[] rgb, int[] target, double tolerance) {
        for (int i = 0; i < 3; i++) {
            int min = (int)(target[i] * (1 - tolerance));
            int max = (int)(target[i] * (1 + tolerance));
            if (rgb[i] <= min || rgb[i] >= max) return false;
        }
        return true;
    }

    public Indexer.ArtifactColor getColor() {
        int[] rgb = {colorSensor.red(), colorSensor.green(), colorSensor.blue()};
        if (checkColorRange(rgb, PURPLE_RGB, PURPLE_TOLERANCE)) return Indexer.ArtifactColor.purple;
        if (checkColorRange(rgb, GREEN_RGB, GREEN_TOLERANCE)) return Indexer.ArtifactColor.green;
        return Indexer.ArtifactColor.unknown;
    }
}
