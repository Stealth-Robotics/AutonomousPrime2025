package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.enums.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class EmergencyResetSpindexer extends SequentialCommandGroup {

    /** Sets all spindexer slots to Artifact.EMPTY and then spins to each, recalculating the artifact inside them **/
    public EmergencyResetSpindexer(SpindexerSubsystem spindexer, IntakeSubsystem intake) {
        addCommands(
                new InstantCommand(() -> spindexer.setArtifactsInSpindexerManually(Artifact.EMPTY, Artifact.EMPTY, Artifact.EMPTY)),
                spindexer.rotateToSlotNumber(1),
                new WaitCommand(500),
                new InstantCommand(() -> spindexer.intakeArtifact(intake.getSensedArtifact())),
                spindexer.rotateToSlotNumber(2),
                new WaitCommand(500),
                new InstantCommand(() -> spindexer.intakeArtifact(intake.getSensedArtifact())),
                spindexer.rotateToSlotNumber(3),
                new WaitCommand(500),
                new InstantCommand(() -> spindexer.intakeArtifact(intake.getSensedArtifact()))
        );
    }
}
