package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.LeaveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.pedroOperation;
import org.firstinspires.ftc.teamcode.storage.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class NearAuto extends StealthOpMode {
    FollowerSubsystem follower;
    IntakeSubsystem intake;
    VisionSubsystem vision;
    ShooterSubsystem shooter;
    TurretSubsystem turret;
    SpindexerSubsystem spindexer;
    pedroOperation p;
    ElapsedTime time;
    static Pose startPose, startIntakePose, finishIntakePose, shootPose;
    static PathChain startToIntake, intakePath, intakeToShoot;
    @Override
    public void initialize(){
        time = new ElapsedTime(0);
        follower = new FollowerSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        p = new pedroOperation(Alliance.get());
        startPose = p.Pose(26.77,130.35,Math.toRadians(180));
        startIntakePose = p.Pose(42.868,84.685,Math.toRadians(180));
        finishIntakePose = p.Pose(24.496,84.510,Math.toRadians(180));
        shootPose = p.Pose(33.594,108.481,Math.toRadians(126));
        startToIntake = follower.pathBuilder()
                .addPath(new BezierCurve(startPose, p.Pose(81,88), startIntakePose))
                .setLinearHeadingInterpolation(startPose.getHeading(),startIntakePose.getHeading())
                .build();
        intakePath = follower.pathBuilder()
                .addPath(new BezierLine(startIntakePose, finishIntakePose))
                .setLinearHeadingInterpolation(startIntakePose.getHeading(),finishIntakePose.getHeading())
                .build();
        intakeToShoot = follower.pathBuilder()
                .addPath(new BezierLine(finishIntakePose, shootPose))
                .setLinearHeadingInterpolation(finishIntakePose.getHeading(), shootPose.getHeading())
                .build();
    }
    @Override
    public Command getAutoCommand(){
        return new SequentialCommandGroup(
                new InstantCommand(() -> time.reset()),
                new InstantCommand(() -> follower.setStartingPose(startPose)),
                follower.followPath(startToIntake,false),
                new InstantCommand(() -> intake.start()),
                follower.followPath(intakePath,false),
                new InstantCommand(() -> intake.stop()),
                follower.followPath(intakeToShoot,false),
                new RunCommand(() -> spindexer.rotateArtifactToShoot(Artifact.PURPLE)),
                new RunCommand(() -> shooter.spinToVelocity(10000)),
                new LeaveCommand(time, follower)
        );
    }
}
