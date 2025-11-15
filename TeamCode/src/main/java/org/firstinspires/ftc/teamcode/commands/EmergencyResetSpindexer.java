package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.ArtifactSource;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class EmergencyResetSpindexer extends SequentialCommandGroup {

    /** Sets all spindexer slots to Artifact.EMPTY and then spins to each, recalculating the artifact inside them **/
    public EmergencyResetSpindexer(SpindexerSubsystem spindexer, IntakeSubsystem intake) {
        addCommands(
                new InstantCommand(() -> spindexer.setArtifactsInSpindexerManually(Artifact.EMPTY, Artifact.EMPTY, Artifact.EMPTY)),
                spindexer.rotateEmptyToIntake(),
                new WaitUntilCommand(() -> intake.getSensedArtifact() != Artifact.EMPTY),
                new InstantCommand(() -> spindexer.updateArtifactState(intake.getSensedArtifact(), ArtifactSource.INTAKE)),
                spindexer.rotateEmptyToIntake(),
                new WaitUntilCommand(() -> intake.getSensedArtifact() != Artifact.EMPTY),
                new InstantCommand(() -> spindexer.updateArtifactState(intake.getSensedArtifact(), ArtifactSource.INTAKE)),
                spindexer.rotateEmptyToIntake(),
                new WaitUntilCommand(() -> intake.getSensedArtifact() != Artifact.EMPTY),
                new InstantCommand(() -> spindexer.updateArtifactState(intake.getSensedArtifact(), ArtifactSource.INTAKE))
        );
    }
}
