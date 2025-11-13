package org.stealthrobotics.library;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;

public class HSVDetector {
    public static final double RED = 0, GREEN = 120, BLUE = 240;
    public static final double PURPLE = 290;
    public static boolean hueInProximity(RevColorSensorV3 colorSensor, double hueTarget, double threshold){
        double hue = rgbToHsv(colorSensor.red(), colorSensor.green(), colorSensor.blue())[0];
        return Math.abs(hueTarget - hue) <= threshold;
    }
    public static boolean valueBelowThreshold(RevColorSensorV3 colorSensor, double minTrue){
        return rgbToHsv(colorSensor.red(), colorSensor.green(), colorSensor.blue())[2] > minTrue;
    }

    static double[] rgbToHsv(double red, double green, double blue){
        double[] hsv = new double[3];
        double r = red / 256, g = green / 256, b = blue / 256;
        double cMax = 0, cMin = 1;
        if(r > cMax) cMax = r;
        if(g > cMax) cMax = g;
        if(b > cMax) cMax = b;
        if(r < cMin) cMin = r;
        if(g < cMin) cMin = g;
        if(b < cMin) cMin = b;
        double delta = cMax - cMin;
        if(delta == 0) hsv[0] = 0;
        else if(cMax == r) hsv[0] = 60 * (((g - b) / delta) % 6);
        else if(cMax == g) hsv[0] = 60 * ((b - r) / delta + 2);
        else if(cMax == b) hsv[0] = 60 * ((r - g) / delta + 4);
        if(cMax == 0) hsv[1] = 0;
        else hsv[1] = delta / cMax;
        hsv[2] = cMax;
        return hsv;
    }
}
