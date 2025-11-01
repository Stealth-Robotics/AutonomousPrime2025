package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class IntakeFromShooterCommand extends SequentialCommandGroup {
    public IntakeFromShooterCommand(ShooterSubsystem shooter, SpindexerSubsystem spindexer, IntakeSubsystem intake, Trigger stopIntaking) {
        if (!spindexer.isFull()) {
            addCommands(
                    spindexer.rotateEmptyToShooter(),
                    shooter.setIntaking(true),
                    intake.startReverse(),
                    new WaitUntilCommand(stopIntaking::get),
                    spindexer.intakeArtifact(Artifact.PURPLE, true), // ! Hardcoded to be Artifact.PURPLE
                    intake.stop(),
                    shooter.stop()
            );
        }
        addRequirements(intake, spindexer);
    }
}