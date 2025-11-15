package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.function.BooleanSupplier;

public class ShootRapidCommand extends SequentialCommandGroup {
    public ShootRapidCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer, BooleanSupplier end) {
        addCommands(
                new RepeatCommand(
                        new SequentialCommandGroup(
                        spindexer.rotateClosestArtifactToShoot(),
                                new ShootCommand(shooter, intake, spindexer)
                        ).interruptOn(end)

                )
        );
    }
}