package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.util.AutoBuilder;
import org.firstinspires.ftc.teamcode.autonomous.util.NearAuto;
import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;

public class NearSingleCycle extends NearAuto {
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                builder.fromStartToShoot(autoType),
                new AutonomousShootCommand(robot, follower),
                builder.cycle(AutoBuilder.PresetLocation.NEAR, autoType),
                new AutonomousShootCommand(robot, follower),
                new WaitCommand(200),
                builder.parkNear(),
                new SaveSubsystemData(robot, follower)
        );
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
