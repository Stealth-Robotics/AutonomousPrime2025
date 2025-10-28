package org.firstinspires.ftc.teamcode.storage;

import com.pedropathing.geometry.Pose;

public class PoseTracker {
    private static Pose pose = new Pose();

    public static void updateEstimatedPose(Pose newPose, boolean updateHeading) {
        if (updateHeading)
            pose = newPose;
        else
            pose = new Pose(newPose.getX(), newPose.getY(), pose.getHeading());
    }

    /**
     * @return The robot's estimated pose x (inches), y (inches), heading (radians)
     */
    public static Pose getEstimatedPose() {
        return pose;
    }
}
