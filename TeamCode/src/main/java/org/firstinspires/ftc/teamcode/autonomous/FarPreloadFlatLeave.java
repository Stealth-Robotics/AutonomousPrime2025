package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.util.FarAuto;
import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SeeMotifCommand;

public class FarPreloadFlatLeave extends FarAuto {
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new SeeMotifCommand(robot, follower),
                builder.fromStartToShootFar(),
                new AutonomousShootCommand(robot, follower),
                new WaitCommand(300),
                builder.farFlatLeave(),
                new InstantCommand(() -> robot.turret.switchToHome())
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedFarPreloadFlatLeave", group = "Red")
    public static class RedFarPreloadFlatLeave extends FarPreloadFlatLeave {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueFarPreloadFlatLeave", group = "Blue")
    public static class BlueFarPreloadFlatLeave extends FarPreloadFlatLeave {
    }
}
