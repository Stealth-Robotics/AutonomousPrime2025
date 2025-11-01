package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;

import java.util.function.DoubleSupplier;

public class IndexerDefaultCommand extends CommandBase {
    private final SpindexerSubsystem spindexer;
    private final DoubleSupplier power;

    public IndexerDefaultCommand(SpindexerSubsystem spindexer, DoubleSupplier power) {
        this.spindexer = spindexer;
        this.power = power;

        addRequirements(spindexer);
    }

    @Override
    public void execute() {
        spindexer.setPower(power.getAsDouble());
    }
}
