package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer) {
        addCommands(
                shooter.spinToVelocity(),
                intake.start(),
                intake.deployLoader(),
                spindexer.shootArtifact(), //Update spindexer slot states
                new WaitCommand(500), //TODO Tune to smallest value that still works
                intake.retractLoader()
        );

        addRequirements(intake, shooter);
    }
}
