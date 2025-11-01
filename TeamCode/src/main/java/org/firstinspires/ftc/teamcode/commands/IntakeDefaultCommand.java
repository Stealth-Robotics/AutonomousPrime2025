package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeDefaultCommand extends CommandBase {
    private final IntakeSubsystem intake;
    private final DoubleSupplier intakeSupplier;

    public IntakeDefaultCommand(IntakeSubsystem intake, DoubleSupplier intakeSupplier) {
        this.intake = intake;
        this.intakeSupplier = intakeSupplier;

        addRequirements(intake);
    }

    @Override
    public void execute() {
        intake.setPower(intakeSupplier.getAsDouble());
    }
}
