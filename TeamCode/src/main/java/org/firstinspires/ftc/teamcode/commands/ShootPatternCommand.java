package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
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
        addCommands(
                new InstantCommand(() -> {
                    if (spindexer.hasMotifColors()) {
                        Artifact[] pattern = Motif.getPattern();
                        for (int i = 0; i < 3; i++) {
                            boolean finalShot = (i == 2);
                            addCommands(
                                    spindexer.rotateArtifactToShoot(pattern[i]),
                                    new ShootCommand(shooter, intake, spindexer, () -> finalShot),
                                    new WaitCommand(500) //Space shots to allow motif to be scored
                            );
                        }
                    }
                    else {
                        //Otherwise shoot any excess artifacts
                        ArrayList<Artifact> extras = spindexer.getExtraArtifacts();
                        for (int i = 0; i < extras.size(); i++) {
                            boolean finalShot = (i == extras.size()-1);
                            addCommands(
                                    spindexer.rotateArtifactToShoot(extras.get(i)),
                                    new ShootCommand(shooter, intake, spindexer, () -> finalShot)
                            );
                        }
                    }
                })
        );
    }
}
