package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

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
}
