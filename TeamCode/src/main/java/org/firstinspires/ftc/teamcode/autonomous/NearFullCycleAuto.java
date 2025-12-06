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

public class NearFullCycleAuto extends StealthOpMode {
    private FollowerSubsystem follower;
    private AutoBuilder builder;
    private RobotSystem robot;
    private Pose startPose;

    @Override
    public void initialize(){
        follower = new FollowerSubsystem(hardwareMap);
        robot = new RobotSystem(hardwareMap);
        builder = new AutoBuilder(robot, follower);

        startPose = builder.CLOSE_START_POSE;
        robot.drive.resetPosAndIMU();

        robot.drive.setPose(startPose);
        follower.setStartingPose(startPose);
    }

    @Override
    public Command getAutoCommand(){
        return new SequentialCommandGroup(
                new AutonomousShootCommand(robot),
                builder.cycle(AutoBuilder.preset.NEAR,true,true),
                new AutonomousShootCommand(robot),
                builder.cycle(AutoBuilder.preset.MIDDLE, true, true),
                new AutonomousShootCommand(robot),
                builder.cycle(AutoBuilder.preset.FAR, true, true),
                builder.parkNear(),
                new SaveSubsystemData(robot),
                new EndOpModeCommand(this)
        );
    }
    @SuppressWarnings("unused")
    @Autonomous(name = "RedNearFullCycle", group = "Red")
    public static class RedNearFullCycle extends NearFullCycleAuto {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueNearFullCycle", group = "Blue")
    public static class BlueNearFullCycle extends NearFullCycleAuto {
    }
}
