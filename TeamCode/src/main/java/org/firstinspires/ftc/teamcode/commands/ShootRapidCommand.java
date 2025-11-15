package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.stealthrobotics.library.Commands;

import java.util.function.BooleanSupplier;

public class ShootRapidCommand extends SequentialCommandGroup {
    public ShootRapidCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer, BooleanSupplier end) {
        addCommands(
                new RepeatCommand(
                        new SequentialCommandGroup(
                        spindexer.rotateClosestArtifactToShoot(),
                                new ShootCommand(shooter, intake, spindexer, () -> (spindexer.size() == 1))
                        ).interruptOn(end)

                )
        );
//        addCommands(
//                new ConditionalCommand(new SequentialCommandGroup(
//                        new ConditionalCommand(
//                                new SequentialCommandGroup(
//                                        spindexer.rotateClosestArtifactToShoot(),
//                                        new ShootCommand(shooter, intake, spindexer, () -> false),
//                                        new ConditionalCommand(
//                                                new SequentialCommandGroup(
//                                                        spindexer.rotateClosestArtifactToShoot(),
//                                                        new ShootCommand(shooter, intake, spindexer, () -> false),
//                                                        new ConditionalCommand(
//                                                                new SequentialCommandGroup(
//                                                                        spindexer.rotateClosestArtifactToShoot(),
//                                                                        new ShootCommand(shooter, intake, spindexer, () -> false)
//                                                                ), new SequentialCommandGroup(
//                                                                spindexer.rotateClosestArtifactToShoot(),
//                                                                new ShootCommand(shooter, intake, spindexer, () -> true)
//                                                        ), () -> spindexer.size() > 1)
//                                                ), new SequentialCommandGroup(
//                                                spindexer.rotateClosestArtifactToShoot(),
//                                                new ShootCommand(shooter, intake, spindexer, () -> true)
//                                        ), () -> spindexer.size() > 1)
//                        ), new SequentialCommandGroup(
//                                spindexer.rotateClosestArtifactToShoot(),
//                                new ShootCommand(shooter, intake, spindexer, () -> true)
//                        ), () -> spindexer.size() == 1)
//                ), new InstantCommand(), () -> spindexer.size() > 0)
//        );


    }
}