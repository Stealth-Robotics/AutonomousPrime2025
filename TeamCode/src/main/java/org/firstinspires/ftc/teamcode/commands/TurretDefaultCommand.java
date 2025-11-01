package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AprilTagPose;
import org.firstinspires.ftc.teamcode.PoseTracker;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.stealthrobotics.library.Alliance;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;


public class TurretDefaultCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final Trigger stopTrigger;
    private final Pose goalPose;

    public TurretDefaultCommand(TurretSubsystem turret, Alliance alliance, Trigger stopTrigger) {
        this.turret = turret;
        this.stopTrigger = stopTrigger;

        if (alliance.equals(Alliance.BLUE)) goalPose = AprilTagPose.BLUE_GOAL.pose;
        else goalPose = AprilTagPose.RED_GOAL.pose;

        addRequirements(turret);
    }

    @Override
    public void execute() {
        if (!stopTrigger.get()) {
            Pose currentPose = PoseTracker.getEstimatedPose();
            double targetAngle = Math.atan2(goalPose.getY() - currentPose.getY(), goalPose.getX() - currentPose.getX());
            turret.setTargetAngle(MathFunctions.clamp(
                    AngleUnit.normalizeDegrees(AngleUnit.RADIANS.toDegrees(currentPose.getHeading() - targetAngle)), -50, 50
            ));
            turret.setPower(turret.calculatePIDPower());

            telemetry.addData("target angle", AngleUnit.RADIANS.toDegrees(targetAngle));
        }
    }
}
