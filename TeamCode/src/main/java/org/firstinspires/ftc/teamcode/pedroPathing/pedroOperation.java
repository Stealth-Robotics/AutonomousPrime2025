package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

import org.stealthrobotics.library.Alliance;

public class pedroOperation {
    //Methods for swapping points based on alliance
    public static Pose AlliancePose(Alliance a, Pose inputPose) {
        if(a.equals(Alliance.BLUE)){
            return new Pose(144 - inputPose.getX(), inputPose.getY(), 180 - inputPose.getHeading());
        } else {
            return inputPose;
        }
    }
    public static Pose AlliancePose(Alliance a, double x, double y, double heading) {
        if(a.equals(Alliance.BLUE)){
            return new Pose(144 - x, y, 180 - heading);
        } else {
            return new Pose(x, y, heading);
        }
    }

    //Added alliance property for efficient implementation
    Alliance a;
    public pedroOperation(Alliance a){
        this.a = a;
    }

    public Pose Pose(double x, double y, double heading) {
        return AlliancePose(this.a, x, y, heading);
    }
    public Pose Pose(double x, double y){
        return AlliancePose(this.a, new Pose(x, y));
    }
}
