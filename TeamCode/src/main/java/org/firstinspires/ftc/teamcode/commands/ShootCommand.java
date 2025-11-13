package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.ArtifactSource;
import org.firstinspires.ftc.teamcode.IntakeState;
import org.firstinspires.ftc.teamcode.ShooterState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.function.BooleanSupplier;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer, BooleanSupplier finalShot) {
        SequentialCommandGroup shootCommand = new SequentialCommandGroup();
        shootCommand.addCommands(
                new InstantCommand(() -> shooter.setState(ShooterState.SHOOT)),
                new WaitUntilCommand(shooter::atVelocity),
                new InstantCommand(() -> intake.setState(IntakeState.TRANSFERRING)),
                new InstantCommand(() -> spindexer.updateArtifactState(Artifact.EMPTY, ArtifactSource.SHOOTER)),
                new WaitCommand(200), //Delay to wait for ball to shoot out
                new InstantCommand(() -> intake.setState(IntakeState.OUTTAKE))
        );

        if (finalShot.getAsBoolean()) {
            shootCommand.addCommands(
                    new InstantCommand(() -> shooter.setState(ShooterState.IDLE)),
                    new InstantCommand(() -> intake.setState(IntakeState.IDLE))
            );
        }

        addCommands(shootCommand);
    }
}
