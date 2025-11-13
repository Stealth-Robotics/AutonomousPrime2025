package org.stealthrobotics.library;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

public class HSVDetector {
    /**
     *
     * @param colorSensor A RevColorSensorV3 object to provide the color input
     * @param hueTarget The target hue value
     * @param threshold The allowed deviation from the hueTarget
     * @return True if the hueTarget is within the threshold, otherwise return false
     */
    public static boolean hueInProximity(RevColorSensorV3 colorSensor, double hueTarget, double threshold) {
        NormalizedRGBA normalizedColors = colorSensor.getNormalizedColors();
        double hue = rgbToHsv(normalizedColors.red, normalizedColors.green, normalizedColors.blue)[0];
        return Math.abs(hueTarget - hue) <= threshold;
    }

    /**
     * @param r The red value normalized between [0, 1)
     * @param g The green value normalized between [0, 1)
     * @param b The blue value normalized between [0, 1)
     * @return The converted HSV (hue, saturation, value) values stored in an array in that order
     */
    public static double[] rgbToHsv(float r, float g, float b) {
        double cMax = Math.max(r, Math.max(g, b));
        double cMin = Math.min(r, Math.min(g, b));
        double delta = cMax - cMin;

        double hue = 0, saturation, value;

        if (delta == 0)
            hue = 0;
        else if (cMax == r)
            hue = 60 * (((((g - b) / delta) % 6) + 6) % 6);
        else if (cMax == g)
            hue = 60 * ((b - r) / delta + 2);
        else if (cMax == b)
            hue = 60 * ((r - g) / delta + 4);

        if (cMax == 0)
            saturation = 0;
        else
            saturation = delta / cMax;

        value = cMax;
        return new double[] {hue, saturation, value};
    }
}
