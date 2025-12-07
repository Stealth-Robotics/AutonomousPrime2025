package org.firstinspires.ftc.teamcode.autonomous.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.AutoToTeleopData;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;
import org.firstinspires.ftc.teamcode.enums.Artifact;
import org.firstinspires.ftc.teamcode.systems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class FarAuto extends StealthOpMode {
    protected FollowerSubsystem follower;
    protected AutoBuilder builder;
    protected RobotSystem robot;

    protected final AutoBuilder.AutoType autoType = AutoBuilder.AutoType.FAR;

    protected Pose startPose;

    @Override
    public void initialize() {
        follower = new FollowerSubsystem(hardwareMap);
        robot = new RobotSystem(hardwareMap);
        builder = new AutoBuilder(robot, follower);

        startPose = builder.FAR_START_POSE;

        follower.setStartingPose(startPose);
    }
}
