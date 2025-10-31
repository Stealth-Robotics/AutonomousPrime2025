package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@SuppressWarnings("FieldCanBeLocal")
public class IntakeSubsystem extends StealthSubsystem {
    private final DcMotorEx transferMotor;
    private final Servo loaderServo;
    private final RevColorSensorV3 colorSensor;

    //TODO: Tune
    private final double LOADER_DEPLOYED_POSITION = 0.0;
    private final double LOADER_RETRACTED_POSITION = 0.0;

    //Distance has to be less than this to look for a artifact
    private final double DISTANCE_THRESHOLD_INCHES = 0.0;

    private final double TRANSFER_SPEED = 1.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        transferMotor = hardwareMap.get(DcMotorEx.class, "transferMotor");
        loaderServo = hardwareMap.get(Servo.class, "loaderServo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
    }

    public Artifact detectedArtifact() {
        Artifact artifact = Artifact.EMPTY;
        if (colorSensor.getDistance(DistanceUnit.INCH) < DISTANCE_THRESHOLD_INCHES) {
            int r = colorSensor.red(), g = colorSensor.green(), b = colorSensor.blue();
            if ((r < 0 && r > 0) && (g > 0 && g < 0) && (b > 0 && b < 0)) {
                artifact = Artifact.GREEN;
            }
            else if ((r < 0 && r > 0) && (g > 0 && g < 0) && (b > 0 && b < 0)) {
                artifact = Artifact.PURPLE;
            }
        }
        return artifact;
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

    public Command stop() {
        return this.runOnce(() -> setPower(0.0));
    }

    private void setPower(double power) {
        transferMotor.setPower(power);
    }

    @Override
    public void periodic() {
        // ! For tuning only (delete after)
        telemetry.addData("red", colorSensor.red());
        telemetry.addData("green", colorSensor.green());
        telemetry.addData("blue", colorSensor.blue());

        telemetry.addData("distance", colorSensor.getDistance(DistanceUnit.INCH));
    }
}
