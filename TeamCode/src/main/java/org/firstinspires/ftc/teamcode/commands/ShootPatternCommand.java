package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.ArrayList;

public class ShootPatternCommand extends SequentialCommandGroup {
    public ShootPatternCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer) {
        if (spindexer.hasMotifColors()) {
            //Has the colors to shoot a full motif
            Artifact[] pattern = Motif.getPattern();
            for (int i = 0; i < 3; i++) {
                addCommands(
                        spindexer.rotateArtifactToShoot(pattern[i]),
                        new ShootCommand(shooter, intake, spindexer),
                        new WaitCommand(500) //Space shots to allow motif to be scored
                );
            }
        }
        else {
            //Otherwise shoot any colors that stop us from intaking artifacts for a motif
            ArrayList<Artifact> extras = spindexer.getExtraArtifacts();
            for (Artifact extra : extras) {
                addCommands(
                        spindexer.rotateArtifactToShoot(extra),
                        new ShootCommand(shooter, intake, spindexer),
                        new WaitCommand(500) //Space shots to allow motif to be scored
                );
            }
        }

        addCommands(new EndShootingCommand(shooter, intake)); //Stop all subsystems from spinning

        addRequirements(shooter, spindexer, intake);
    }
}
