package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.PoseTracker;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.stealthrobotics.library.Alliance;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TurretDefaultCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final Pose goalPose;
    private double targetAngle = 0;
    private final Pose BlueGoalPose = new Pose(-58.3727,-55.6425);
    private final Pose RedGoalPose = new Pose(-58.3727, 55.6425);

    public TurretDefaultCommand(TurretSubsystem turret, Alliance alliance) {
        this.turret = turret;
        if (alliance.equals(Alliance.BLUE)) {
            goalPose = BlueGoalPose;
        } else {
            goalPose = RedGoalPose;
        }
        addRequirements(turret);
    }

    @Override
    public void execute() {
        Pose currentPose = PoseTracker.getEstimatedPose();
        targetAngle = Math.atan2(goalPose.getY() - currentPose.getY(), goalPose.getX() - currentPose.getX());
        turret.setTarget(targetAngle-currentPose.getHeading());
    }
}
