package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.IntakeState;
import org.stealthrobotics.library.ColorRange;
import org.stealthrobotics.library.ColorSensorMatcher;
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

    private final Debouncer purpleColorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);
    private final Debouncer greenColorDebouncer = new Debouncer(0.1, Debouncer.DebounceType.kRising);

    private final double DISTANCE_THRESHOLD_MM = 100.0;

//    private final ColorRange greenArtifactRange = new ColorRange(0, 90, 80, 1000, 0, 1000);
//    private final ColorRange purpleArtifactRange = new ColorRange(40, 1000, 0, 120, 0, 1000);
    private final double GREEN_HUE = 120, PURPLE_HUE = 290;
    private final double HUE_THRESHOLD_DEGREES = 40;

    private IntakeState state = IntakeState.IDLE;

    private Artifact sensedArtifact = Artifact.EMPTY;

    public static double LOADER_DEPLOYED_POSITION = 0.25;
    public static double LOADER_RETRACTED_POSITION = 0.04;

    public static double OPERATING_SPEED = 1.0;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        loaderServo = hardwareMap.get(Servo.class, "loaderServo");
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setState(IntakeState newState) {
        state = newState;
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
            //Debounce color sensor for artifact colors
//            boolean isPurple = purpleColorDebouncer.calculate(ColorSensorMatcher.inRange(colorSensor, purpleArtifactRange));
//            boolean isGreen = greenColorDebouncer.calculate(ColorSensorMatcher.inRange(colorSensor, greenArtifactRange));
            boolean isPurple = purpleColorDebouncer.calculate(HSVDetector.hueInProximity(colorSensor, PURPLE_HUE, HUE_THRESHOLD_DEGREES));
            boolean isGreen = greenColorDebouncer.calculate(HSVDetector.hueInProximity(colorSensor, GREEN_HUE, HUE_THRESHOLD_DEGREES));

            if (isPurple) sensedArtifact = Artifact.PURPLE;
            else if (isGreen) sensedArtifact = Artifact.GREEN;
            else sensedArtifact = Artifact.EMPTY;
        }

        //State-machine
        if (state == IntakeState.INTAKE) {
            setPower(OPERATING_SPEED);
            loaderServo.setPosition(LOADER_RETRACTED_POSITION);
        }
        else if (state == IntakeState.OUTTAKE) {
            setPower(-OPERATING_SPEED);
            loaderServo.setPosition(LOADER_RETRACTED_POSITION);
        }
        else if (state == IntakeState.TRANSFERRING) {
            setPower(-OPERATING_SPEED);
            loaderServo.setPosition(LOADER_DEPLOYED_POSITION);
        }
        else {
            setPower(0.0);
            loaderServo.setPosition(LOADER_RETRACTED_POSITION);
        }

        telemetry.addLine("----intake----");
        telemetry.addData("state", state);
        telemetry.addData("detected artifact", getSensedArtifact());
    }
}
