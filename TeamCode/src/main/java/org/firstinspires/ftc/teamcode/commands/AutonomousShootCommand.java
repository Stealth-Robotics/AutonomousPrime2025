package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.systems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class AutonomousShootCommand extends SequentialCommandGroup {
    public AutonomousShootCommand(RobotSystem robot, FollowerSubsystem follower) {
        addCommands(
                new InstantCommand(() -> PoseEstimator.getInstance().update(follower.getPose())),
                new WaitCommand(300),
                robot.setRobotState(RobotSystem.RobotState.PRE_PATTERN),
                new WaitUntilCommand(() -> robot.getState() == RobotSystem.RobotState.IDLE),
                //Just in case we don't have pattern colors just shoot rapid
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                robot.setRobotState(RobotSystem.RobotState.PRE_RAPID),
                                new WaitUntilCommand(() -> robot.getState() == RobotSystem.RobotState.IDLE)
                        ),
                        new InstantCommand(),
                        () -> !robot.spindexer.isEmpty()
                )
        );
    }
}
