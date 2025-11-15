package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.ArtifactSource;
import org.firstinspires.ftc.teamcode.IntakeState;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.ShooterState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

//public class ShootPatternCommand extends CommandBase {
//    private final ShooterSubsystem shooter;
//    private final SpindexerSubsystem spindexer;
//    private final IntakeSubsystem intake;
//
//    public ShootPatternCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer) {
//        this.shooter = shooter;
//        this.intake = intake;
//        this.spindexer = spindexer;
//    }
//
//    @Override
//    public void initialize() {
//        SequentialCommandGroup shootPatternSequence = new SequentialCommandGroup();
//
//        if (spindexer.hasMotifColors()) {
//            Artifact[] pattern = Motif.getPattern();
//            for (int i = 0; i < 3; i++) {
//                boolean finalShot = (i == 2);
//                shootPatternSequence.addCommands(
//                        spindexer.rotateArtifactToShoot(pattern[i]),
//                        new ShootCommand(shooter, intake, spindexer, () -> finalShot),
//                        new WaitCommand(500) //Space shots to allow motif to be scored
//                );
//            }
//        }
//        else {
//            //Otherwise shoot any excess artifacts
//            ArrayList<Artifact> extras = spindexer.getExtraArtifacts();
//            for (int i = 0; i < extras.size(); i++) {
//                boolean finalShot = (i == extras.size()-1);
//                shootPatternSequence.addCommands(
//                        spindexer.rotateArtifactToShoot(extras.get(i)),
//                        new ShootCommand(shooter, intake, spindexer, () -> finalShot)
//                );
//            }
//        }
//
//        shootPatternSequence.schedule();
//    }
//
//    @Override
//    public boolean isFinished() {
//        return false;
//    }
//}
public class ShootPatternCommand extends SequentialCommandGroup {
    public ShootPatternCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer) {
        if (spindexer.hasMotifColors()) {
            //Shoot motif pattern
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
    }
}
