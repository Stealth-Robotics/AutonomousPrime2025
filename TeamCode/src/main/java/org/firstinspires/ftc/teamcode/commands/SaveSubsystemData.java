package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.AutoToTeleopData;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class SaveSubsystemData extends SequentialCommandGroup {
    public SaveSubsystemData(DriveSubsystem drive, SpindexerSubsystem spindexer, TurretSubsystem turret) {
        addCommands(
                new InstantCommand(() -> AutoToTeleopData.pinpointPose = new Pose2D(DistanceUnit.INCH, drive.getPoseX(), drive.getPoseY(), AngleUnit.DEGREES, AngleUnit.RADIANS.toDegrees(drive.getHeading()))),
                new InstantCommand(() -> AutoToTeleopData.spindexerTicks = spindexer.getTicks()),
                new InstantCommand(() -> AutoToTeleopData.turretTicks = turret.getRawTicks()),
                new InstantCommand(() -> {
                    Artifact[] artifacts = spindexer.getCurrentArtifacts();
                    AutoToTeleopData.slot1Artifact = artifacts[0];
                    AutoToTeleopData.slot2Artifact = artifacts[1];
                    AutoToTeleopData.slot3Artifact = artifacts[2];
                })
        );
    }
}
