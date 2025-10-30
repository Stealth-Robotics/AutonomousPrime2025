package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public enum AprilTagPose {
    BLUE_GOAL(new Pose(16.3575, 130.3727)),
    RED_GOAL(new Pose(127.6425, 130.3727));

    public final Pose pose;
    AprilTagPose(Pose pose) {
        this.pose = pose;
    }
}
