package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.AutoToTeleopData;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class LoadSubsystemData extends SequentialCommandGroup {
    public LoadSubsystemData(DriveSubsystem drive, SpindexerSubsystem spindexer, TurretSubsystem turret) {
        addCommands(
                new InstantCommand(() -> drive.setPose(AutoToTeleopData.pinpointPose)),
                new InstantCommand(() -> spindexer.setEncoderOffset(AutoToTeleopData.spindexerTicks)),
                new InstantCommand(() -> turret.setEncoderOffset(AutoToTeleopData.turretTicks)),
                new InstantCommand(() -> spindexer.setStartingConfig(AutoToTeleopData.slot1Artifact, AutoToTeleopData.slot2Artifact, AutoToTeleopData.slot3Artifact))
        );
    }
}
