package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class ShootRapidCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final SpindexerSubsystem spindexer;
    private final IntakeSubsystem intake;

    private boolean done = false;

    public ShootRapidCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer) {
        this.shooter = shooter;
        this.intake = intake;
        this.spindexer = spindexer;
    }

    @Override
    public void initialize() {
        SequentialCommandGroup shootRapidSequence = new SequentialCommandGroup();

        int size = spindexer.size();
        for (int i = 0; i < size; i++) {
            boolean finalShot = (i == size - 1);
            shootRapidSequence.addCommands(
                    spindexer.rotateClosestArtifactToShoot().withTimeout(1500),
                    new ShootCommand(shooter, intake, spindexer, () -> finalShot)
            );
        }

        shootRapidSequence.andThen(new InstantCommand(() -> done = true)).schedule();
    }

    @Override
    public boolean isFinished() {
        return done;
    }
}