package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PoseSupplier;
import org.firstinspires.ftc.teamcode.TurretState;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;
import org.firstinspires.ftc.teamcode.pedroPathing.AlliancePoseFlipper;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class FarAuto extends StealthOpMode {
    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private TurretSubsystem turret;
    private SpindexerSubsystem spindexer;
    private LimelightSubsystem limelight;

    //Align with outer edge of one of the two center tiles (turret faces forward)
    private Pose2D startPose = new Pose2D(DistanceUnit.INCH, 56.287, 9.147, AngleUnit.DEGREES, 90);

    @Override
    public void initialize() {
        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap, new PoseSupplier(() -> drive.getPoseX(), () -> drive.getPoseY(), () -> AngleUnit.RADIANS.toDegrees(drive.getHeading())));
        spindexer = new SpindexerSubsystem(hardwareMap, intake);
        limelight = new LimelightSubsystem(hardwareMap);

        //Flip starting pose if on red alliance side of the field
        if (Alliance.get() == Alliance.RED)
            startPose =  AlliancePoseFlipper.flip(startPose);
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                //Set pinpoint's position
                new WaitUntilCommand(() -> drive.isPPReady()),
                new InstantCommand(() -> drive.setPose(startPose)),

                //Move away from start pose so can see obelisk and also shoot better
                new InstantCommand(() -> drive.drive(0, 0.5, 0)).andThen(new WaitCommand(300).andThen(new InstantCommand(() -> drive.stop()))),

                //Motif
                new InstantCommand(() -> turret.setState(TurretState.OBELISK)),
                new WaitCommand(500),
                new InstantCommand(() -> turret.setState(TurretState.GOAL)),

                //Do auto stuff here (shoot motif (if not null) and move from launch zone)

                //Leave launch zone for points
                new InstantCommand(() -> drive.drive(0, 0.5, 0)).andThen(new WaitCommand(400).andThen(new InstantCommand(() -> drive.stop()))),

                //End auto (save states)
                new WaitCommand(200),
                new SaveSubsystemData(drive, spindexer, turret)
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedFarAuto", group = "Red")
    public static class RedFarAuto extends FarAuto {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueFarAuto", group = "Blue")
    public static class BlueFarAuto extends FarAuto {
    }
}
