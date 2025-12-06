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

    private Pose robotPose = new Pose(0, 0,0);

    private final double INCHES_FROM_ORIGIN_TO_TURRET = 3; //Distance from the robot's origin to the rotation point of the turret

    private static final Pose OBELISK_POSE = new Pose(72, 144);

    private static final Pose BLUE_GOAL_POSE = new Pose(16.3575, 130.3727);
    private static final Pose RED_GOAL_POSE = new Pose(127.6425, 130.3727);

    private static Pose goalPose = null;

    public PoseEstimator() {
        if (Alliance.get() == Alliance.BLUE)
            goalPose = BLUE_GOAL_POSE;
        else
            goalPose = RED_GOAL_POSE;
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

    public void update(Pose updatedRobotPose) {
        robotPose = updatedRobotPose;
    }

    /**
     * @return The required turret angle to point at the obelisk for motif detection in autonomous
     */
    public double getObeliskTurretTargetAngle() {
        double robotHeading = robotPose.getHeading();
        double turretX = robotPose.getX() + INCHES_FROM_ORIGIN_TO_TURRET * cos(robotHeading);
        double turretY = robotPose.getY() + INCHES_FROM_ORIGIN_TO_TURRET * sin(robotHeading);

        double targetAngleDegrees = AngleUnit.RADIANS.toDegrees(Math.atan2(OBELISK_POSE.getY() - turretY, OBELISK_POSE.getX() - turretX));
        return AngleUnit.normalizeDegrees(AngleUnit.RADIANS.toDegrees(robotPose.getHeading()) - targetAngleDegrees);
    }

    public double getTurretTargetAngle() {
        if (Alliance.get() == Alliance.BLUE)
            goalPose = BLUE_GOAL_POSE;
        else
            goalPose = RED_GOAL_POSE;

        double robotHeading = robotPose.getHeading();
        double turretX = robotPose.getX() + INCHES_FROM_ORIGIN_TO_TURRET * cos(robotHeading);
        double turretY = robotPose.getY() + INCHES_FROM_ORIGIN_TO_TURRET * sin(robotHeading);

        double targetAngleDegrees = AngleUnit.RADIANS.toDegrees(Math.atan2(goalPose.getY() - turretY, goalPose.getX() - turretX));
        return AngleUnit.normalizeDegrees(AngleUnit.RADIANS.toDegrees(robotPose.getHeading()) - targetAngleDegrees);
    }

    public double getDistanceFromGoal() {
        return sqrt(pow((robotPose.getX() - goalPose.getX()), 2) + pow((robotPose.getY() - goalPose.getY()), 2));
    }
}
