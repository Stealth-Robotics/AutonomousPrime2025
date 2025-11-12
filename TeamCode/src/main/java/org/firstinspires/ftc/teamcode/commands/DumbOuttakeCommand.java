package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.IntakeState;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.function.BooleanSupplier;

public class DumbOuttakeCommand extends SequentialCommandGroup {
    public DumbOuttakeCommand(IntakeSubsystem intake, BooleanSupplier stop) {
        addCommands(
                new InstantCommand(() -> intake.setState(IntakeState.OUTTAKE)),
                new WaitUntilCommand(stop),
                new InstantCommand(() -> intake.setState(IntakeState.IDLE))
        );
    }
}
