package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.systems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class SeeMotifCommand extends SequentialCommandGroup {
    public SeeMotifCommand(RobotSystem robot, FollowerSubsystem follower) {
        addCommands(
                new WaitCommand(500),
                new InstantCommand(() -> PoseEstimator.getInstance().update(follower.getPose())),
                new InstantCommand(() -> robot.turret.switchToObelisk()),
                new WaitCommand(1000), //Pause to see motif
                new InstantCommand(() -> robot.turret.switchToOdometryControl())
        );
    }
}
