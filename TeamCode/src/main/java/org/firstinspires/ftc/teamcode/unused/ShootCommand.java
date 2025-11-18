//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitCommand;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.Artifact;
//import org.firstinspires.ftc.teamcode.ArtifactSource;
//import org.firstinspires.ftc.teamcode.IntakeState;
//import org.firstinspires.ftc.teamcode.ShooterState;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
//
//import java.util.function.BooleanSupplier;
//
//public class ShootCommand extends SequentialCommandGroup {
//    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer) {
//        addCommands(
//                new InstantCommand(() -> shooter.setState(ShooterState.SHOOT)),
//                new WaitUntilCommand(shooter::atVelocity).withTimeout(4000),
//                new InstantCommand(() -> intake.setState(IntakeState.TRANSFERRING_UP)),
//                new WaitCommand(600),
//                new InstantCommand(() -> intake.setState(IntakeState.TRANSFERRING_IDLE)),
//                new WaitCommand(200), //Wait for loader arm to get out of way of spindexer
//                new InstantCommand(() -> spindexer.updateArtifactState(Artifact.EMPTY, ArtifactSource.SHOOTER)),
//                new InstantCommand(() -> shooter.setState(ShooterState.IDLE)),
//                new InstantCommand(() -> intake.setState(IntakeState.IDLE))
//        );
//    }
//}