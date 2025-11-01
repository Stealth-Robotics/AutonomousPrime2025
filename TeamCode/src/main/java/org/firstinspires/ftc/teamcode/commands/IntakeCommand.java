package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand(IntakeSubsystem intake, SpindexerSubsystem spindexer) {
        if (!spindexer.isFull()) {
            addCommands(
                    spindexer.rotateEmptyToIntake(),
                    intake.start(),
                    new WaitUntilCommand(() -> intake.detectedArtifact() != Artifact.EMPTY),
                    spindexer.intakeArtifact(intake.detectedArtifact(), false), //Update spindexer slot states
                    intake.stop()
            );
        }
        addRequirements(intake, spindexer);
    }
}
