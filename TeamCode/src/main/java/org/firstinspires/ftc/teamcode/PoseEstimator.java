package org.firstinspires.ftc.teamcode;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.stealthrobotics.library.Alliance;

public class PoseEstimator {
    private static PoseEstimator INSTANCE = null;

    private Pose robotPoseNew = null;
    private Pose robotPose = new Pose();

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

    public Pose getRobotPose() {
        return robotPose;
    }

    public static PoseEstimator getInstance() {
        if (INSTANCE == null) {
            INSTANCE = new PoseEstimator();
            return INSTANCE;
        }
        return INSTANCE;
    }

    //Update via camera with new apriltag data
    public void updateWithNewPose(Pose newEstimation) {
        robotPoseNew = newEstimation;
    }

    //Update via pinpoint based off of robot velocity
    public boolean update(Pose updatedRobotPose) {
        robotPose = updatedRobotPose;
        if (robotPoseNew != null) {
            robotPose = robotPoseNew;
            robotPoseNew = null;
            return true;
        }
        return false;
    }

    public double getTurretTargetAngle() {
        double targetAngleDegrees = AngleUnit.RADIANS.toDegrees(Math.atan2(goalPose.getY() - robotPose.getY(), goalPose.getX() - robotPose.getX()));
        return AngleUnit.normalizeDegrees(robotPose.getHeading() - targetAngleDegrees);
    }

    public double getDistanceFromGoal() {
        if (robotPose == null || goalPose == null)
            return 0;
        return sqrt(pow((robotPose.getX() - goalPose.getX()), 2) + pow((robotPose.getY() - goalPose.getY()), 2));
    }
}
