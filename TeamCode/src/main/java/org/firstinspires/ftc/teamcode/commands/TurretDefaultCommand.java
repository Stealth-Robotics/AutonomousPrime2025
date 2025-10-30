package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.AprilTagPose;
import org.firstinspires.ftc.teamcode.PoseTracker;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.stealthrobotics.library.Alliance;

public class TurretDefaultCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final Pose goalPose;


    public TurretDefaultCommand(TurretSubsystem turret, Alliance alliance) {
        this.turret = turret;

        if (alliance.equals(Alliance.BLUE)) goalPose = AprilTagPose.BLUE_GOAL.pose;
        else goalPose = AprilTagPose.RED_GOAL.pose;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose currentPose = PoseTracker.getEstimatedPose();
        double targetAngle = Math.atan2(goalPose.getY() - currentPose.getY(), goalPose.getX() - currentPose.getX());
        turret.setTargetAngle(targetAngle - currentPose.getHeading());
    }
}
