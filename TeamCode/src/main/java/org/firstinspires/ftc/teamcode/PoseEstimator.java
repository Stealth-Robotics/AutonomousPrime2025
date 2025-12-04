package org.firstinspires.ftc.teamcode;

import static java.lang.Math.cos;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.stealthrobotics.library.Alliance;

public class PoseEstimator {
    private static PoseEstimator INSTANCE = null;

    private Pose robotPoseNew = null;
    private Pose robotPose = new Pose(0, 0, 0);

    private final double INCHES_FROM_ORIGIN_TO_TURRET = 3; //Distance from the robot's origin to the rotation point of the turret

    private static final Pose BLUE_GOAL_POSE = new Pose(16.3575, 130.3727);
    private static final Pose RED_GOAL_POSE = new Pose(127.6425, 130.3727);

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
        if (robotPoseNew != null) {
            robotPose = robotPoseNew;
            robotPoseNew = null;
            return true;
        }

        robotPose = updatedRobotPose;
        return false;
    }

    public double getTurretTargetAngle() {
        double robotHeading = robotPose.getHeading();
        double turretX = robotPose.getX() + INCHES_FROM_ORIGIN_TO_TURRET * cos(robotHeading);
        double turretY = robotPose.getY() + INCHES_FROM_ORIGIN_TO_TURRET * sin(robotHeading);

        double targetAngleDegrees = AngleUnit.RADIANS.toDegrees(Math.atan2(goalPose.getY() - turretY, goalPose.getX() - turretX));
        return AngleUnit.normalizeDegrees(AngleUnit.RADIANS.toDegrees(robotPose.getHeading()) - targetAngleDegrees);
    }

    public double getDistanceFromGoal() {
        return sqrt(pow((robotPose.getX() - goalPose.getX()), 2) + pow((robotPose.getY() - goalPose.getY()), 2));
    }

    public Pose getGoalPose() {
        return goalPose;
    }
}
