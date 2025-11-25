package org.firstinspires.ftc.teamcode.enums;

public enum LEDState {
    OFF(0.0),
    RED(0.3),
    ORANGE(0.333),
    YELLOW(0.388),
    SAGE(0.444),
    GREEN(0.5),
    AZURE(0.555),
    BLUE(0.611),
    INDIGO(0.666),
    VIOLET(0.722),
    WHITE(1.0);

    private final double colorValue;

    LEDState(double colorValue) {
        this.colorValue = colorValue;
    }

    public double getColorValue() {
        return colorValue;
    }
}