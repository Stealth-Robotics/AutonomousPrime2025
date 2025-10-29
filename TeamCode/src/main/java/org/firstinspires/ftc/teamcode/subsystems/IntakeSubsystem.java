package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.storage.Artifact;
import org.stealthrobotics.library.StealthSubsystem;

public class IntakeSubsystem extends StealthSubsystem {
    private final DcMotorEx transferMotor;
    private final RevColorSensorV3 colorSensor;

    private final double transferSpeed = 1.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    }

    public Artifact detectedArtifact() {
        return true; //TODO Implement
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
