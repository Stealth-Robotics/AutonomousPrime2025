package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.enums.Artifact;
import org.firstinspires.ftc.teamcode.enums.IntakeState;
import org.stealthrobotics.library.HSVDetector;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.math.filter.Debouncer;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import android.graphics.Color;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class IntakeSubsystem extends StealthSubsystem {
    private final DcMotorEx intakeMotor;
    private final Servo loaderServo;
    private final RevColorSensorV3 colorSensor;

    private IntakeState state = IntakeState.IDLE;

    private Artifact sensedArtifact = Artifact.EMPTY;

    private final Debouncer purpleColorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    private final Debouncer greenColorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private final double DISTANCE_THRESHOLD_MM = 90.0;

    private final double GREEN_HUE = 150, PURPLE_HUE = 190;
    private final double GREEN_HUE_THRESHOLD = 20, PURPLE_HUE_THRESHOLD = 20;

    private final double STALL_THRESHOLD = 3.8;

    private final double LOADER_DEPLOYED_POSITION = 0.5;
    private final double LOADER_RETRACTED_POSITION = 0.015;

    private final double OPERATING_SPEED = 1.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        loaderServo = hardwareMap.get(Servo.class, "loaderServo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public boolean isStalled() {
        return intakeMotor.getCurrent(CurrentUnit.AMPS) >= STALL_THRESHOLD;
    }

    public Command setState(IntakeState newState) {
        return new InstantCommand(() -> state = newState);
    }

    public IntakeState getState() {
        return state;
    }

    public Artifact getSensedArtifact() {
        return sensedArtifact;
    }

    public void setPower(double power) {
        intakeMotor.setPower(power);
    }

    @Override
    public void periodic() {
        //Only look for artifacts if something is in front of color sensor
        if (colorSensor.getDistance(DistanceUnit.MM) < DISTANCE_THRESHOLD_MM) {
            //Debounce color sensor for artifact colors (helps to remove false positives caused by lighting conditions and interference)
            boolean isPurple = purpleColorDebouncer.calculate(HSVDetector.hueInProximity(colorSensor, PURPLE_HUE, PURPLE_HUE_THRESHOLD));
            boolean isGreen = greenColorDebouncer.calculate(HSVDetector.hueInProximity(colorSensor, GREEN_HUE, GREEN_HUE_THRESHOLD));

            if (isPurple)
                sensedArtifact = Artifact.PURPLE;
            else if (isGreen)
                sensedArtifact = Artifact.GREEN;
            else
                sensedArtifact = Artifact.EMPTY;
        }
        else sensedArtifact = Artifact.EMPTY;

        //State-machine
        if (state == IntakeState.INTAKE) {
            setPower(OPERATING_SPEED);
            loaderServo.setPosition(LOADER_RETRACTED_POSITION);
        }
        else if (state == IntakeState.OUTTAKE) {
            setPower(-OPERATING_SPEED);
            loaderServo.setPosition(LOADER_RETRACTED_POSITION);
        }
        else if (state == IntakeState.TRANSFER) {
            setPower(OPERATING_SPEED);
            loaderServo.setPosition(LOADER_DEPLOYED_POSITION);
        }
        else {
            setPower(0);
            loaderServo.setPosition(LOADER_RETRACTED_POSITION);
        }
    }
}
