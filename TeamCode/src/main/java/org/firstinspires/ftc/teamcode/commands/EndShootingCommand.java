package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

public class EndShootingCommand extends SequentialCommandGroup {

    public EndShootingCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
        addCommands(shooter.stop(), intake.stop());
        addRequirements(intake, shooter);
    }
}
