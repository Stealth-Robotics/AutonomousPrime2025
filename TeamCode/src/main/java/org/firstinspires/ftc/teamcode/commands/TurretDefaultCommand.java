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
    public void execute() {
        double targetAngle = 0.0; //Set based off of apriltag
        turret.setTarget(targetAngle);
        turret.setPower(turret.pidCalculate());
    }
}
