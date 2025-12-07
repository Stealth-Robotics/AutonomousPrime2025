package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import android.sax.StartElementListener;

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

    public Pose getPose() {
        return follower.getPose();
    }

    public PathBuilder pathBuilder() {
        return follower.pathBuilder();
    }

    public Command followPath(PathChain path, boolean holdPoint) {
        return this.runOnce(() -> follower.followPath(path, holdPoint))
                .andThen(new WaitUntilCommand(() -> !follower.isBusy()));
    }

    public Command followPath(PathChain path, double maxSpeed, boolean holdPoint) {
        return this.runOnce(() -> follower.followPath(path, maxSpeed, holdPoint))
                .andThen(new WaitUntilCommand(() -> !follower.isBusy()));
    }

    public boolean isBusy() {
        return follower.isBusy();
    }

    @Override
    public void periodic() {
        follower.update();
    }
}
