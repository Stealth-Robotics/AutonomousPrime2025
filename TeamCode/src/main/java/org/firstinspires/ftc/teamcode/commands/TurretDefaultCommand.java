package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.storage.PoseTracker;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.stealthrobotics.library.Alliance;

public class TurretDefaultCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final Pose goalPose;

    //In pedro coordinates
    private final Pose BLUE_GOAL_POSE = new Pose(16.3575, 130.3727);
    private final Pose RED_GOAL_POSE = new Pose(127.6425, 130.3727);


    public TurretDefaultCommand(TurretSubsystem turret, Alliance alliance) {
        this.turret = turret;

        if (alliance.equals(Alliance.BLUE)) goalPose = BLUE_GOAL_POSE;
        else goalPose = RED_GOAL_POSE;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose currentPose = PoseTracker.getEstimatedPose();
        double targetAngle = Math.atan2(goalPose.getY() - currentPose.getY(), goalPose.getX() - currentPose.getX());
        turret.setTargetAngle(targetAngle - currentPose.getHeading());
    }
}
