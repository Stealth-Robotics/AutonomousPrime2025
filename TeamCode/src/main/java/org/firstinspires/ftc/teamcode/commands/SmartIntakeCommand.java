package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.ArtifactSource;
import org.firstinspires.ftc.teamcode.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.function.BooleanSupplier;

public class SmartIntakeCommand extends SequentialCommandGroup {
    /** Will intake and reschedule command if there is more space in spindexer.
     * You must provide a boolean supplier that stops the process (aka a timeout in auto, or a trigger release in teleop).
     */

    public SmartIntakeCommand(IntakeSubsystem intake, SpindexerSubsystem spindexer, BooleanSupplier stop) {
        addCommands(
                new InstantCommand(() -> {
                    int artifactCount = spindexer.size();
                    for (int i = 0; i < artifactCount; i++) {
                        addCommands(
                                spindexer.rotateEmptyToIntake(),
                                new InstantCommand(() -> intake.setState(IntakeState.INTAKE)),
                                new WaitUntilCommand(() -> intake.getSensedArtifact() != Artifact.EMPTY),
                                new InstantCommand(() -> spindexer.updateArtifactState(intake.getSensedArtifact(), ArtifactSource.INTAKE))
                        );
                    }

                    //Stop intaking if done
                    addCommands(new InstantCommand(() -> intake.setState(IntakeState.IDLE)));
                })
        );

        raceWith(
                new WaitUntilCommand(stop).andThen(new InstantCommand(() -> intake.setState(IntakeState.IDLE)))
        );
    }
}
