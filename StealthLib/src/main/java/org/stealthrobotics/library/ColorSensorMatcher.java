package org.stealthrobotics.library;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class ColorSensorMatcher {
    public static boolean inRange(RevColorSensorV3 colorSensor, ColorRange range) {
        int r = colorSensor.red(), g = colorSensor.green(), b = colorSensor.blue();
        return (r >= range.getRedRange()[0] && r <= range.getRedRange()[1] &&
                g >= range.getGreenRange()[0] && g <= range.getGreenRange()[1] &&
                b >= range.getBlueRange()[0] && b <= range.getBlueRange()[1]);
    }
}
