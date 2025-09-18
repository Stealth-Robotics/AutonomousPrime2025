package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

public class TransferSubsystem extends StealthSubsystem {
    private final DcMotorEx transferMotor;
    private final double transferSpeed = 1.0;

    public TransferSubsystem(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
    }

    public Command start() {
        return this.runOnce(() -> setPower(transferSpeed));
    }

    public Command stop() {
        return this.runOnce(() -> setPower(0));
    }

    private void setPower(double power) {
        transferMotor.setPower(power);
    }
}
