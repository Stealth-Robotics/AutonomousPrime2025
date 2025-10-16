package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class PoseTracker {
    private static Pose pose;

    public static void updateEstimatedPose(Pose newPose) {
        pose = newPose;
    }

    //Pedro's coordinate system is at 0, 0 in the bottom left hand corner
    public static Pose getEstimatedPose() {
        return pose;
    }
}
