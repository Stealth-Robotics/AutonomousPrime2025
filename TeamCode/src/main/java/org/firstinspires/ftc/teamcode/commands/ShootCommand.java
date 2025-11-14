package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
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

public class ShootCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final SpindexerSubsystem spindexer;
    private final IntakeSubsystem intake;
    private final BooleanSupplier finalShot;

    private boolean done = false;

    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer, BooleanSupplier finalShot) {
        this.shooter = shooter;
        this.intake = intake;
        this.spindexer = spindexer;
        this.finalShot = finalShot;
    }

    @Override
    public void initialize() {
        SequentialCommandGroup shootSequence = new SequentialCommandGroup();

        shootSequence.addCommands(
                new InstantCommand(() -> shooter.setState(ShooterState.SHOOT)),
                new WaitUntilCommand(shooter::atVelocity).withTimeout(4000),
                new InstantCommand(() -> intake.setState(IntakeState.TRANSFERRING_UP)),
                new WaitCommand(1000),
                new InstantCommand(() -> spindexer.updateArtifactState(Artifact.EMPTY, ArtifactSource.SHOOTER)),
                new InstantCommand(() -> intake.setState(IntakeState.TRANSFERRING_IDLE)),
                new WaitCommand(500) //Wait for loader arm to get out of way of spindexer
        );

        if (finalShot.getAsBoolean()) {
            shootSequence.addCommands(
                    new InstantCommand(() -> shooter.setState(ShooterState.IDLE)),
                    new InstantCommand(() -> intake.setState(IntakeState.IDLE))
            );
        }

        shootSequence.andThen(new InstantCommand(() -> done = true)).schedule();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
