package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.enums.LEDState;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.math.filter.Debouncer;

import java.util.function.BooleanSupplier;

public class LEDSubsystem extends StealthSubsystem {
    private final Servo led;
    private LEDState currentState = LEDState.OFF;
    private LEDState newState = LEDState.OFF;

    private final Debouncer timingDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);

    public LEDSubsystem(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "led");
    }

    public void setState(LEDState newState) {
        this.newState = newState;
    }

    public LEDState getState() {
        return currentState;
    }

    @Override
    public void periodic() {
        if (timingDebouncer.calculate(currentState != newState)) {
            led.setPosition(newState.getColorValue());
            currentState = newState;
        }
    }
}
