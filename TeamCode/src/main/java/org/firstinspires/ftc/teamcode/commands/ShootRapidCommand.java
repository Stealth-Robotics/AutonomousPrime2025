package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class ShootRapidCommand extends SequentialCommandGroup {
    public ShootRapidCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer) {
        for (int i = 0; i < spindexer.gamepieceCount(); i++) {
            addCommands(
                    spindexer.rotateClosestArtifactToShoot(),
                    new ShootCommand(shooter, intake, spindexer)
            );
        }
        addCommands(new EndShootingCommand(shooter, intake));

        addRequirements(shooter, spindexer);
    }
}
