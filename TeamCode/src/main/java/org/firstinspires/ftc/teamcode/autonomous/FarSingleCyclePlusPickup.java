package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.util.AutoBuilder;
import org.firstinspires.ftc.teamcode.autonomous.util.FarAuto;
import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;
import org.firstinspires.ftc.teamcode.commands.SeeMotifCommand;

public class FarSingleCyclePlusPickup extends FarAuto {
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new SeeMotifCommand(robot, follower),
                builder.fromStartToShootFar(),
                new AutonomousShootCommand(robot, follower),
                builder.cycle(AutoBuilder.PresetLocation.FAR, autoType),
                new AutonomousShootCommand(robot, follower),
                builder.halfCycle(AutoBuilder.PresetLocation.MIDDLE, autoType),
                new SaveSubsystemData(robot, follower)
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedFarSingleCyclePlusPickup", group = "Red")
    public static class RedFarSingleCyclePlusPickup extends FarSingleCyclePlusPickup {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueFarSingleCyclePlusPickup", group = "Blue")
    public static class BlueFarSingleCyclePlusPickup extends FarSingleCyclePlusPickup {
    }
}
