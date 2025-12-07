package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.util.AutoBuilder;
import org.firstinspires.ftc.teamcode.autonomous.util.NearAuto;
import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;
import org.firstinspires.ftc.teamcode.commands.SeeMotifCommand;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;

public class NearSingleCycle extends NearAuto {
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                builder.fromStartToShootNear(),
                new SeeMotifCommand(robot, follower),
                new AutonomousShootCommand(robot, follower),
                (builder.cycle(AutoBuilder.PresetLocation.NEAR, autoType).andThen(new AutonomousShootCommand(robot, follower))),
                robot.setRobotState(RobotSystem.RobotState.IDLE),
                new WaitCommand(200),
                builder.parkNear(),
                new InstantCommand(() -> robot.turret.switchToHome())
        );
    }

    @Override
    public void bruh() {
        robot.turret.switchToHome();
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedNearSingleCycle", group = "Red")
    public static class RedNearSingleCycle extends NearSingleCycle {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueNearSingleCycle", group = "Blue")
    public static class BlueNearSingleCycle extends NearSingleCycle {
    }
}
