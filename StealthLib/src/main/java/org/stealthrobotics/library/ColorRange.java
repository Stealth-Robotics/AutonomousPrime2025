package org.stealthrobotics.library;

public class ColorRange {
    private final int redBoundLow, redBoundHigh, greenBoundLow, greenBoundHigh, blueBoundLow, blueBoundHigh;

    public ColorRange(int redBoundLow, int redBoundHigh, int greenBoundLow, int greenBoundHigh, int blueBoundLow, int blueBoundHigh) {
        this.redBoundLow = redBoundLow;
        this.redBoundHigh = redBoundHigh;
        this.greenBoundLow = greenBoundLow;
        this.greenBoundHigh = greenBoundHigh;
        this.blueBoundLow = blueBoundLow;
        this.blueBoundHigh = blueBoundHigh;
    }

    public int[] getRedRange() {
        return new int[] {redBoundLow, redBoundHigh};
    }

    public int[] getGreenRange() {
        return new int[] {greenBoundLow, greenBoundHigh};
    }

    public int[] getBlueRange() {
        return new int[] {blueBoundLow, blueBoundHigh};
    }
}
