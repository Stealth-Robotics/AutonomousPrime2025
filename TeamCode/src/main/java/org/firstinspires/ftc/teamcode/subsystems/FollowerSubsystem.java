package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.stealthrobotics.library.StealthSubsystem;

public class FollowerSubsystem extends StealthSubsystem {
    Follower follower;
    public FollowerSubsystem(HardwareMap hardwareMap){
        follower = Constants.createFollower(hardwareMap);
    }
    public void setStartingPose(Pose p){
        follower.setStartingPose(p);
    }
    public Pose getPose(){
        return follower.getPose();
    }
    public PathBuilder pathBuilder(){
        return follower.pathBuilder();
    }

    public Command followPath(PathChain p, boolean holdPoint){
        return this.runOnce(()-> follower.followPath(p,holdPoint))
                .andThen(new WaitUntilCommand(()-> !follower.isBusy()));
    }
}
