package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;

public class TurretDefaultCommand extends CommandBase {
    private final TurretSubsystem turret;

    public TurretDefaultCommand(TurretSubsystem turret) {
        this.turret = turret;
        addRequirements(turret);
    }

    @Override
    public void initialize() {
        turret.resetEncoder();
    }

    @Override
    public void execute() {
        turret.setPower(turret.pidCalc());
    }
}
