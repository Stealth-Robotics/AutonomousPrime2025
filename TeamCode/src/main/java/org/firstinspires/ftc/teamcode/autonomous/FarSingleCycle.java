package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.util.AutoBuilder;
import org.firstinspires.ftc.teamcode.autonomous.util.FarAuto;
import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;
import org.firstinspires.ftc.teamcode.commands.SeeMotifCommand;

public class FarSingleCycle extends FarAuto {
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new SeeMotifCommand(robot, follower),
                builder.fromStartToShootFar(),
                new AutonomousShootCommand(robot, follower),
                builder.cycle(AutoBuilder.PresetLocation.FAR, autoType),
                new AutonomousShootCommand(robot, follower),
                new WaitCommand(200),
                builder.parkFar(),
                new InstantCommand(() -> robot.turret.switchToHome())
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedFarSingleCycle", group = "Red")
    public static class RedFarSingleCycle extends FarSingleCycle {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueFarSingleCycle", group = "Blue")
    public static class BlueFarSingleCycle extends FarSingleCycle {
    }
}
