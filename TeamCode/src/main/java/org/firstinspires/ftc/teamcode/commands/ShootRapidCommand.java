package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class ShootRapidCommand extends CommandBase {
    private final SequentialCommandGroup shootRapidSequence = new SequentialCommandGroup();

    private final ShooterSubsystem shooter;
    private final SpindexerSubsystem spindexer;
    private final IntakeSubsystem intake;

    public ShootRapidCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer) {
        this.shooter = shooter;
        this.intake = intake;
        this.spindexer = spindexer;
    }

    @Override
    public void initialize() {
        int size = spindexer.size();
        for (int i = 0; i < size; i++) {
            boolean finalShot = (i == size - 1);
            shootRapidSequence.addCommands(
                    spindexer.rotateClosestArtifactToShoot(),
                    new ShootCommand(shooter, intake, spindexer, () -> finalShot)
            );
        }

        shootRapidSequence.schedule();
    }

    @Override
    public boolean isFinished() {
        return shootRapidSequence.isFinished();
    }
}
