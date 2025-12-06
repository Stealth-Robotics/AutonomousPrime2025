package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.util.NearAuto;
import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;

public class NearPreload extends NearAuto {
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                builder.fromStartToShootNear(),
                new AutonomousShootCommand(robot, follower),
                builder.parkNear(),
                new SaveSubsystemData(robot, follower)
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedNearPreload", group = "Red")
    public static class RedNearPreload extends NearPreload {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueNearPreload", group = "Blue")
    public static class BlueNearPreload extends NearPreload {
    }
}
