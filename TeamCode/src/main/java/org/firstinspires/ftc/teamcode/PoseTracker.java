package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import org.stealthrobotics.library.Alliance;

public class PoseTracker {
    private static Pose pose = new Pose();
    private static Pose allianceGoal = new Pose();

    public static void setAlliance() {
        if (Alliance.get() == Alliance.BLUE) {
            allianceGoal = AprilTagPose.BLUE_GOAL.pose;
        }
        else {
            allianceGoal = AprilTagPose.RED_GOAL.pose;
        }
    }

    public static void updateEstimatedPose(Pose newPose, boolean updateHeading) {
        if (updateHeading)
            pose = newPose;
        else
            pose = new Pose(newPose.getX(), newPose.getY(), pose.getHeading());
    }

    public static double getDistanceFromGoal() {
        return Math.sqrt(Math.pow(allianceGoal.getX() - pose.getX(), 2) + Math.pow(allianceGoal.getY() - pose.getY(), 2));
    }

    /**
     * @return The robot's estimated pose x (inches), y (inches), heading (radians)
     */
    public static Pose getEstimatedPose() {
        return pose;
    }
}
