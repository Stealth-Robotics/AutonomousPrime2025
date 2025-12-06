package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.autonomous.util.FarAuto;
import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;
import org.firstinspires.ftc.teamcode.commands.SeeMotifCommand;

public class FarPreload extends FarAuto {
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new SeeMotifCommand(robot, follower),
                builder.fromStartToShootFar(),
                new AutonomousShootCommand(robot, follower),
                builder.parkFar(),
                new SaveSubsystemData(robot, follower)
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedFarPreload", group = "Red")
    public static class RedFarPreload extends FarPreload {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueFarPreload", group = "Blue")
    public static class BlueFarPreload extends FarPreload {
    }
}
