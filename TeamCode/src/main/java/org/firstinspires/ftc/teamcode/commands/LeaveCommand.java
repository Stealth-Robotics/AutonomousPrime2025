package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;

public class LeaveCommand extends SequentialCommandGroup {
    public final double TIME_THRESHOLD = 28;
    Pose leavePose;
    PathChain leavePath;
    private Pose nearestLeavePose(Pose currentPose){
        if(currentPose.getX() < 72 && currentPose.getY() > -currentPose.getX() + 120 ||
           currentPose.getX() > 72 && currentPose.getY() < -currentPose.getX() + 120){
            return new Pose(currentPose.getX(), 120 - currentPose.getX(), Math.toRadians(45));
        } else if (currentPose.getX() >= 72 && currentPose.getY() > currentPose.getX() - 24 ||
                   currentPose.getX() <= 72 && currentPose.getY() < currentPose.getX() - 24) {
            return new Pose(currentPose.getX(), currentPose.getX() - 24, Math.toRadians(45));
        } else {
            return currentPose;
        }
    }
    public LeaveCommand(ElapsedTime time, FollowerSubsystem follower){
        if(time.seconds() > TIME_THRESHOLD){
            Pose currentPose = follower.getPose();
            leavePose = nearestLeavePose(currentPose);
            leavePath = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose,leavePose))
                    .setLinearHeadingInterpolation(currentPose.getHeading(),leavePose.getHeading())
                    .build();
            addCommands(follower.followPath(leavePath, false));
        }
    }
}
