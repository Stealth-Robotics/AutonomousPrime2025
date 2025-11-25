package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.enums.Artifact;
import org.firstinspires.ftc.teamcode.AutoToTeleopData;
import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;

public class SaveSubsystemData extends SequentialCommandGroup {
    public SaveSubsystemData(RobotSystem robot) {
        addCommands(
                new InstantCommand(() -> AutoToTeleopData.spindexerTicks = robot.spindexer.getTicks()),
                new InstantCommand(() -> AutoToTeleopData.turretTicks = robot.turret.getRawTicks()),
                new InstantCommand(() -> {
                    Artifact[] artifacts = robot.spindexer.getCurrentArtifacts();
                    AutoToTeleopData.slot1Artifact = artifacts[0];
                    AutoToTeleopData.slot2Artifact = artifacts[1];
                    AutoToTeleopData.slot3Artifact = artifacts[2];
                }),
                new InstantCommand(() -> AutoToTeleopData.endOfAutoPose = PoseEstimator.getInstance().getRobotPose())
        );
    }
}
