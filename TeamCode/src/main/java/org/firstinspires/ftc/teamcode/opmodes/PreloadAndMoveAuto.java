package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.AutoToTeleop;
import org.firstinspires.ftc.teamcode.PoseTracker;
import org.firstinspires.ftc.teamcode.commands.AlignTurretCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TurretDefaultCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.pedroOperation;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class PreloadAndMoveAuto extends StealthOpMode {
    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private ShooterSubsystem shooter;
    private TurretSubsystem turret;
    private SpindexerSubsystem spindexer;

    //Flips to alliance relative pose
    private final Pose startPose = pedroOperation.AlliancePose(Alliance.get(), new Pose(57.224, 8.67, Math.toRadians(90)));

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        PoseTracker.updateEstimatedPose(startPose, true);
        PoseTracker.setAlliance();
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> drive.setStartPose(startPose)),
                new AlignTurretCommand(turret, Alliance.get()),
                spindexer.rotateToShoot(SpindexerSubsystem.SpindexerSlot.TWO).withTimeout(1000).andThen(new InstantCommand(() -> spindexer.setPower(0))),
                new WaitCommand(300),
                shooter.toggleSpinning(),
                new WaitCommand(3000),
                new ShootCommand(shooter, intake),
                new WaitCommand(800),
                shooter.toggleSpinning(),
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> drive.drive(-0.5, 0, 0)),
//                        new WaitCommand(900),
//                        new InstantCommand(() -> drive.drive(0, 0, 0))
//                ),
                turret.unWrapFully() // ! VERY IMPORTANT, otherwise turret will commit die
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedPreloadAndMoveAuto", group = "Red")
    public static class RedTestAuto extends PreloadAndMoveAuto {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BluePreloadAndMoveAuto", group = "Blue")
    public static class BlueTestAuto extends PreloadAndMoveAuto {
    }
}
