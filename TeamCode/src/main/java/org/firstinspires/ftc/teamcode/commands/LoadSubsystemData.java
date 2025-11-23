package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.AutoToTeleopData;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class LoadSubsystemData extends SequentialCommandGroup {
    public LoadSubsystemData(RobotSystem robot) {
        addCommands(
                new InstantCommand(() -> robot.drive.resetPosAndIMU()),
                new InstantCommand(() -> robot.spindexer.setEncoderOffset(AutoToTeleopData.spindexerTicks)),
                new InstantCommand(() -> robot.turret.setEncoderOffset(AutoToTeleopData.turretTicks)),
                new InstantCommand(() -> robot.spindexer.setArtifactsInSpindexerManually(AutoToTeleopData.slot1Artifact, AutoToTeleopData.slot2Artifact, AutoToTeleopData.slot3Artifact)),
                new InstantCommand(() -> robot.drive.setPose(AutoToTeleopData.endOfAutoPose))
        );
    }
}
