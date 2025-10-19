package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TurretDefaultCommand extends CommandBase {
    private final TurretSubsystem turret;
    private final DoubleSupplier right;
    private final DoubleSupplier left;
    double targetAngle = 0;

    public TurretDefaultCommand(TurretSubsystem turret, DoubleSupplier right, DoubleSupplier left) {
        this.turret = turret;
        this.right = right;
        this.left = left;
        addRequirements(turret);
    }

    @Override
    public void execute() {
        turret.setPower((left.getAsDouble() - right.getAsDouble()) * 0.5);
    }
}
