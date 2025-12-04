package org.firstinspires.ftc.teamcode.systems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.stealthrobotics.library.StealthSubsystem;

public class FollowerSubsystem extends StealthSubsystem {
    private final Follower follower;

    public FollowerSubsystem(HardwareMap hardwareMap) {
        follower = Constants.createFollower(hardwareMap);
    }

    public void setStartingPose(Pose startPose) {
        follower.setStartingPose(startPose);
    }

    public void setMaxPower(double power) {
        follower.setMaxPower(power);
    }

    public Pose getPose() {
        return follower.getPose();
    }

    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    public Command followPath(PathChain path, boolean holdPoint) {
        return this.runOnce(()-> follower.followPath(path, holdPoint))
                .andThen(new WaitUntilCommand(() -> !follower.isBusy()))
                .andThen(new InstantCommand(() -> setMaxPower(1.0)));
    }

    @Override
    public void periodic() {
        follower.update();
    }
}
