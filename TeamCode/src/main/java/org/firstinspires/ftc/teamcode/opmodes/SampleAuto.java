package org.firstinspires.ftc.teamcode.opmodes;

import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class SampleAuto extends StealthOpMode {
    FollowerSubsystem followerSubsystem;
    IntakeSubsystem intakeSubsystem;
    LimelightSubsystem limelightSubsystem;
    ShooterSubsystem shooterSubsystem;
    SpindexerSubsystem spindexerSubsystem;
    TurretSubsystem turretSubsystem;
    @Override
    public void initialize(){
        followerSubsystem = new FollowerSubsystem(hardwareMap);
        intakeSubsystem = new IntakeSubsystem(hardwareMap);
        limelightSubsystem = new LimelightSubsystem(hardwareMap);
        shooterSubsystem = new ShooterSubsystem(hardwareMap);
        spindexerSubsystem = new SpindexerSubsystem(hardwareMap);

    }
}
