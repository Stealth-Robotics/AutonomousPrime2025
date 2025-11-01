package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.PoseTracker;
import org.firstinspires.ftc.teamcode.commands.AlignTurretCommand;
import org.firstinspires.ftc.teamcode.commands.TurretDefaultCommand;
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

    private final Pose startPose = new Pose(55, 7.9, Math.toRadians(90));

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap, startPose);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        PoseTracker.setAlliance(); // ! Very important for auto and teleop shooting calculations
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> drive.setStartPose(startPose)),
                new AlignTurretCommand(turret, Alliance.get())
//                new WaitCommand(1000), //Variable pause
//                new SequentialCommandGroup(
//                        new InstantCommand(() -> drive.drive(0, 0.5, 0)),
//                        new WaitCommand(800),
//                        new InstantCommand(() -> drive.drive(0, 0, 0))
//                ),
//                turret.unWrapFully() // ! VERY IMPORTANT, otherwise turret will commit die
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
