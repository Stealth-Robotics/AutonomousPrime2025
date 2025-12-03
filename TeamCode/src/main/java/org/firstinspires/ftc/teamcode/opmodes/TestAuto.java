package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.stealthrobotics.library.commands.EndOpModeCommand;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class TestAuto extends StealthOpMode {
    private AutoBuilder pathLibrary;
    private FollowerSubsystem follower;
    private RobotSystem robot;

    private Pose startPose;

    @Override
    public void initialize() {
        follower = new FollowerSubsystem(hardwareMap);
        robot = new RobotSystem(hardwareMap);

        pathLibrary = new AutoBuilder();
        startPose = pathLibrary.FAR_START_POSE;

        //Reset pinpoint
        robot.drive.resetPosAndIMU();

        //For localization
        robot.drive.setPose(startPose);
        follower.setStartingPose(startPose);
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                // PUT AUTONOMOUS SEQUENCE HERE
                new AutonomousShootCommand(robot),
                new SaveSubsystemData(robot),
                new EndOpModeCommand(this)
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedTestAuto", group = "Red")
    public static class RedTestAuto extends TestAuto {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueTestAuto", group = "Blue")
    public static class BlueTestAuto extends TestAuto {
    }
}
