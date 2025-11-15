package org.firstinspires.ftc.teamcode;

public class LatestGoalData {
    private static boolean seesTag = false;
    private static double headingOffsetFromGoal = 0.0;
    private static double distanceFromGoal = 0.0;

    public static boolean canSeeTag() {
        return seesTag;
    }

    public static void tagInvisible() {
        seesTag = false;
    }

    public static void updateGoalData(double newHeadingOffsetFromGoal) {
        seesTag = true;
        headingOffsetFromGoal = newHeadingOffsetFromGoal;
    }

    public static void updateDistanceFromGoal(double distance) {
        distanceFromGoal = distance;
    }

    public static double getHeadingOffsetFromGoal() {
        return headingOffsetFromGoal;
    }

    public static double getDistanceFromGoal() {
        return distanceFromGoal;
    }
}
