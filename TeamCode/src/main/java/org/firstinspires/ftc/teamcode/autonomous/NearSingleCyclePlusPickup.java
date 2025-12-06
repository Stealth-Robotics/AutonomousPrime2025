package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.util.AutoBuilder;
import org.firstinspires.ftc.teamcode.autonomous.util.NearAuto;
import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;

public class NearSingleCyclePlusPickup extends NearAuto {
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                builder.fromStartToShoot(autoType),
                new AutonomousShootCommand(robot, follower),
                builder.cycle(AutoBuilder.PresetLocation.NEAR, autoType),
                new AutonomousShootCommand(robot, follower),
                builder.halfCycle(AutoBuilder.PresetLocation.MIDDLE, autoType),
                new SaveSubsystemData(robot, follower)
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedNearSingleCyclePlusPickup", group = "Red")
    public static class RedNearSingleCyclePlusPickup extends NearSingleCyclePlusPickup {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueNearSingleCyclePlusPickup", group = "Blue")
    public static class BlueNearSingleCyclePlusPickup extends NearSingleCyclePlusPickup {
    }
}
