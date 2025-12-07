package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.util.NearAuto;
import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;
import org.firstinspires.ftc.teamcode.commands.SeeMotifCommand;

public class NearPreloadNoLeave extends NearAuto {
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                builder.fromStartToShootNear(),
                new SeeMotifCommand(robot, follower),
                new AutonomousShootCommand(robot, follower),
                new InstantCommand(() -> robot.turret.switchToHome())
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedNearPreloadNoLeave", group = "Red")
    public static class RedNearPreloadNoLeave extends NearPreloadNoLeave {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueNearPreloadNoLeave", group = "Blue")
    public static class BlueNearPreloadNoLeave extends NearPreloadNoLeave {
    }
}
