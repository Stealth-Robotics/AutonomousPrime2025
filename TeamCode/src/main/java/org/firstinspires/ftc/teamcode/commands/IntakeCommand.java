package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.storage.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class IntakeCommand extends SequentialCommandGroup {

    public IntakeCommand(IntakeSubsystem intake, SpindexerSubsystem spindexer) {
        if (spindexer.hasEmptySlot()) {
            addCommands(
                    spindexer.rotateEmptyToIntake(),
                    intake.start(),
                    new WaitUntilCommand(() -> intake.detectedArtifact() != Artifact.EMPTY),
                    spindexer.updateSlotState(intake.detectedArtifact(), true, false),
                    intake.stop()
            );
        }

        addRequirements(intake, spindexer);
    }
}
