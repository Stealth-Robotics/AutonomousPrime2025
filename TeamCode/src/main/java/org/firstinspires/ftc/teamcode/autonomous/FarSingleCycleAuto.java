package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;
import org.firstinspires.ftc.teamcode.systems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.stealthrobotics.library.commands.EndOpModeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class FarSingleCycleAuto extends StealthOpMode {
    private FollowerSubsystem follower;
    private AutoBuilder builder;
    private RobotSystem robot;
    private Pose startPose;

    @Override
    public void initialize(){
        follower = new FollowerSubsystem(hardwareMap);
        robot = new RobotSystem(hardwareMap);
        builder = new AutoBuilder(robot, follower);

        startPose = builder.FAR_START_POSE;
        robot.drive.resetPosAndIMU();

        robot.drive.setPose(startPose);
        follower.setStartingPose(startPose);
    }

    @Override
    public Command getAutoCommand(){
        return new SequentialCommandGroup(
                new AutonomousShootCommand(robot),
                builder.cycle(AutoBuilder.preset.FAR,false,false),
                new AutonomousShootCommand(robot),
                builder.parkFar(),
                new SaveSubsystemData(robot),
                new EndOpModeCommand(this)
        );
    }
    @SuppressWarnings("unused")
    @Autonomous(name = "RedFarSingleCycle", group = "Red")
    public static class RedFarSingleCycleAuto extends FarSingleCycleAuto {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueFarSingleCycle", group = "Blue")
    public static class BlueFarSingleCycleAuto extends FarSingleCycleAuto {
    }
}
