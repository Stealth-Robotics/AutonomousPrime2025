package org.firstinspires.ftc.teamcode.autonomous.util;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.AutoToTeleopData;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;
import org.firstinspires.ftc.teamcode.enums.Artifact;
import org.firstinspires.ftc.teamcode.systems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class NearAuto extends StealthOpMode {
    protected FollowerSubsystem follower;
    protected AutoBuilder builder;
    protected RobotSystem robot;

    protected Pose startPose;

    protected final AutoBuilder.AutoType autoType = AutoBuilder.AutoType.NEAR;

    @Override
    public void initialize() {
        follower = new FollowerSubsystem(hardwareMap);
        robot = new RobotSystem(hardwareMap);
        builder = new AutoBuilder(robot, follower);

        startPose = builder.CLOSE_START_POSE;

        follower.setStartingPose(startPose);
    }

    public void saveData() {
        AutoToTeleopData.spindexerTicks = robot.spindexer.getTicks();
        AutoToTeleopData.turretTicks = robot.turret.getRawTicks();

        Artifact[] artifacts = robot.spindexer.getCurrentArtifacts();
        AutoToTeleopData.slot1Artifact = artifacts[0];
        AutoToTeleopData.slot2Artifact = artifacts[1];
        AutoToTeleopData.slot3Artifact = artifacts[2];

        AutoToTeleopData.endOfAutoPose = follower.getPose();
    }
}
