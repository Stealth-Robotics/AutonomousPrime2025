//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//import com.arcrobotics.ftclib.command.button.Trigger;
//
//import org.firstinspires.ftc.teamcode.enums.Artifact;
//import org.firstinspires.ftc.teamcode.ArtifactSource;
//import org.firstinspires.ftc.teamcode.enums.IntakeState;
//import org.firstinspires.ftc.teamcode.enums.ShooterState;
//import org.firstinspires.ftc.teamcode.enums.TurretState;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
//
//public class IntakeFromShooterCommand extends SequentialCommandGroup {
//    public IntakeFromShooterCommand(TurretSubsystem turret, ShooterSubsystem shooter, SpindexerSubsystem spindexer, IntakeSubsystem intake, Trigger stop) {
//        if (!spindexer.isFull()) {
//            addCommands(
//                    new InstantCommand(() -> turret.setState(TurretState.HOME)),
//                    spindexer.rotateEmptyToShooter(),
//                    new InstantCommand(() -> shooter.setState(ShooterState.INTAKE)),
//                    new InstantCommand(() -> intake.setState(IntakeState.OUTTAKE)),
//                    new WaitUntilCommand(stop::get),
//                    new InstantCommand(() -> spindexer.updateArtifactState(Artifact.PURPLE, ArtifactSource.SHOOTER)), // ! Hardcoded to be Artifact.PURPLE
//                    new InstantCommand(() -> intake.setState(IntakeState.IDLE)),
//                    new InstantCommand(() -> shooter.setState(ShooterState.IDLE)),
//                    new InstantCommand(() -> turret.setState(TurretState.SEARCH)) //Return to moving turret
//            );
//        }
//    }
//}