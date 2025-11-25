package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

public class AlliancePoseFlipper {
    public static Pose flip(Pose pose) {
        return new Pose(144 - pose.getX(), pose.getY(), 180 - pose.getHeading());
    }
}
