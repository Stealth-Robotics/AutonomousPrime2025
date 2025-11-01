package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class AutoToTeleop {
    private static Pose endOfAutoPose = null;

    public static void setEndOfAutoPose(Pose p) {
        endOfAutoPose = p;
    }

    public static Pose getEndOfAutoPose() {
        return endOfAutoPose;
    }

    public static boolean hasRunAuto() {
        return endOfAutoPose != null;
    }
}
