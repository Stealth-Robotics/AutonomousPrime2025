package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class ShootRapidCommand extends SequentialCommandGroup {
    public ShootRapidCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer) {
        addCommands(
                new InstantCommand(() -> {
                    int size = spindexer.size();
                    for (int i = 0; i < size; i++) {
                        boolean finalShot = (i == size - 1);
                        addCommands(
                                spindexer.rotateClosestArtifactToShoot(),
                                new ShootCommand(shooter, intake, spindexer, finalShot)
                        );
                    }
                })
        );
    }
}
