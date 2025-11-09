package org.firstinspires.ftc.teamcode;

public class RanAuto {
    private static boolean value = false;

    public static void ranAuto() {
        value = true;
    }

    public static boolean didRunAuto() {
        return value;
    }
}
