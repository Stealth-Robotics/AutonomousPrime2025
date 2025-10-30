package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class RapidShootCommand extends SequentialCommandGroup {
    public RapidShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer) {
        for (int i = 0; i < spindexer.gamepieceCount(); i++) {
            addCommands(
                    spindexer.rotateClosestArtifactToShoot(),
                    shooter.spinToVelocity(),
                    intake.start(),
                    intake.deployLoader(),
                    new WaitCommand(1000), //TODO: tune timings
                    intake.retractLoader()
            );
        }

        addCommands(
                shooter.stop(),
                intake.stop()
        );

        addRequirements(shooter, spindexer);
    }
}
