package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(ShooterSubsystem shooter, IntakeSubsystem intake) {
        addCommands(
                intake.start(),
                intake.deployLoader(),
                new WaitCommand(1000),
                intake.retractLoader(),
                intake.stop()
        );
    }
}
