package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class IntakeSubsystem extends StealthSubsystem {
    private final DcMotorEx transferMotor;
    private final Servo loaderServo;

    private final double LOADER_DEPLOYED_POSITION = 0.5;
    private final double LOADER_RETRACTED_POSITION = 0.1;

    //Distance has to be less than this to look for a artifact
    private final double DISTANCE_THRESHOLD_INCHES = 3.5;

    public static double TRANSFER_SPEED = 1.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        loaderServo = hardwareMap.get(Servo.class, "loaderServo");

        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        retractLoader().schedule();
    }

    public Command retractLoader() {
        return this.runOnce(() -> loaderServo.setPosition(LOADER_RETRACTED_POSITION));
    }

    public Command deployLoader() {
        return this.runOnce(() -> loaderServo.setPosition(LOADER_DEPLOYED_POSITION));
    }

    public Command start() {
        return this.runOnce(() -> setPower(TRANSFER_SPEED));
    }

    public Command startReverse() {
        return this.runOnce(() -> setPower(-TRANSFER_SPEED));
    }

    public Command stop() {
        return this.runOnce(() -> setPower(0));
    }

    public void setPower(double power) {
        transferMotor.setPower(power);
    }
}
