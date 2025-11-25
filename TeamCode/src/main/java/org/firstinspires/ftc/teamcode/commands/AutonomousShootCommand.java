package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

public class AutonomousShootCommand extends SequentialCommandGroup {
    public AutonomousShootCommand(RobotSystem robot) {
        addCommands(
                robot.setRobotState(RobotSystem.RobotState.PRE_PATTERN),
                new WaitUntilCommand(() -> robot.getState() == RobotSystem.RobotState.IDLE),
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
