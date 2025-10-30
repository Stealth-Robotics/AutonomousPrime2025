package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class ShootPatternCommand extends SequentialCommandGroup {

    public ShootPatternCommand(ShooterSubsystem shooter, SpindexerSubsystem spindexer) {

        if (spindexer.hasMotifColors) {

        }

        addRequirements(shooter, spindexer);
    }
}
