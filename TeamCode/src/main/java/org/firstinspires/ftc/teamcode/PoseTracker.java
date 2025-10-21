package org.firstinspires.ftc.teamcode;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;

public class PoseTracker {
    private static Pose pose;

    // ? Takes in FTCCoordinates and saves it in PedroCoordinates
    public static void updateEstimatedPose(Pose newPose) {
        pose = FTCCoordinates.INSTANCE.convertToPedro(newPose);
    }

    public static Pose getEstimatedPose() {
        return pose;
    }
}
