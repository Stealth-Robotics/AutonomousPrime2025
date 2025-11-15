package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.stealthrobotics.library.Alliance;

public class AlliancePoseFlipper {
    //Based on the robot's alliance, flip the auto poses or not
    //The poses should be blue alliance by default
    public static void flip(Alliance alliance, Pose[] poses) {
        for (int i = 0; i < poses.length; i++) {
            if (alliance == Alliance.RED) {
                poses[i] = new Pose(144 - poses[i].getX(), poses[i].getY(), Math.PI - poses[i].getHeading());
            }
        }
    }

    public static Pose2D flip(Pose2D pose) {
        return new Pose2D(DistanceUnit.INCH, 144 - pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH), AngleUnit.DEGREES, 180 - pose.getHeading(AngleUnit.DEGREES));
    }
}
