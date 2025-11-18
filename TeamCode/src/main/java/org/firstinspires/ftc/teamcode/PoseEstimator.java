package org.firstinspires.ftc.teamcode;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.stealthrobotics.library.Alliance;

public class PoseEstimator {
    private static PoseEstimator INSTANCE = null;

    private Pose robotPose = null;
    private Pose latestLimelightPose = null;

    private static final Pose BLUE_GOAL_POSE = new Pose();
    private static final Pose RED_GOAL_POSE = new Pose();

    private static Pose goalPose = null;

    public PoseEstimator() {
        if (Alliance.get() == Alliance.BLUE) {
            goalPose = BLUE_GOAL_POSE;
        }
        else {
            goalPose = RED_GOAL_POSE;
        }
    }

    public static PoseEstimator getInstance() {
        if (INSTANCE == null) {
            return new PoseEstimator();
        }
        return INSTANCE;
    }

    public void updateRobotPose(Pose newEstimation) {
        robotPose = newEstimation;
    }

    public void setLimelightPoseUpdate(Pose newPose) {
        latestLimelightPose = newPose;
    }

    public boolean hasLimelightPoseUpdate() {
        return latestLimelightPose != null;
    }

    public Pose getLatestLimelightPose() {
        Pose updatedPose = latestLimelightPose;
        latestLimelightPose = null;
        return updatedPose;
    }

    public double getTurretTargetAngle() {
        double targetAngleDegrees = AngleUnit.RADIANS.toDegrees(Math.atan2(goalPose.getY() - robotPose.getY(), goalPose.getX() - robotPose.getX()));
        return AngleUnit.normalizeDegrees(robotPose.getHeading() - targetAngleDegrees);
    }

    public double getDistanceFromGoal() {
        return sqrt(pow((robotPose.getX() - goalPose.getX()), 2) + pow((robotPose.getY() - goalPose.getY()), 2));
    }
}
