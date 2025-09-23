package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

import java.util.function.DoubleSupplier;

public class ShooterDefaultCommand extends CommandBase {
    private final ShooterSubsystem shooter;
    private final DoubleSupplier trigger;

    public ShooterDefaultCommand(ShooterSubsystem shooter, DoubleSupplier trigger) {
        this.shooter = shooter;
        this.trigger = trigger;

        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setPower(trigger.getAsDouble());
    }
}
