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
    private boolean checkRange(int num, int origin, double tolerance) {
        boolean valid = false;
        int min = (int)(origin * (1 - tolerance));
        int max = (int)(origin * (1 + tolerance));
        if (num > min && num < max) {
            valid = true;
        }
        return valid;
    }

    // checks if the camera sees purple or green and returns -1 (neither found) 0 (purple found) 1 (green found)
    public Indexer.ArtifactColor getColor() {
        Indexer.ArtifactColor foundColor = Indexer.ArtifactColor.unknown;
        int[] rgb = {colorSensor.red(),colorSensor.green(),colorSensor.blue()};
        boolean foundPurple = true;
        for (int i = 0; i < 2; i++) {
            foundPurple = (foundPurple && checkRange(rgb[i],PURPLE_RGB[i],PURPLE_TOLERANCE));
        }
        if (foundPurple) {
            foundColor = Indexer.ArtifactColor.purple;
        } else {
            boolean foundGreen = true;
            for (int i = 0; i < 2; i++) {
                foundGreen = (foundGreen && checkRange(rgb[i],GREEN_RGB[i],GREEN_TOLERANCE));
            }
            if (foundGreen) foundColor = Indexer.ArtifactColor.green;
        }
        return foundColor;
    }
}