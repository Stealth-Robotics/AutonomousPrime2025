package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;

public class PoseTracker {
    private static Pose pose = new Pose();

    public static void updateEstimatedPose(Pose newPose) {
        pose = newPose;
    }

    /**
     * @return The robot's estimated pose x (inches), y (inches), heading (radians)
     */
    public static Pose getEstimatedPose() {
        return pose;
    }
}
