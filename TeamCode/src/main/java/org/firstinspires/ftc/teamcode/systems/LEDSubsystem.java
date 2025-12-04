package org.firstinspires.ftc.teamcode.systems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.enums.LEDState;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.math.filter.Debouncer;

import java.util.function.BooleanSupplier;

public class LEDSubsystem extends StealthSubsystem {
    private final Servo led;
    private final BooleanSupplier velocityDebouncer;

    private final Debouncer stateDebouncer = new Debouncer(0.25, Debouncer.DebounceType.kRising);

    public LEDSubsystem(HardwareMap hardwareMap, BooleanSupplier velocityDebouncer) {
        led = hardwareMap.get(Servo.class, "led");
        this.velocityDebouncer = velocityDebouncer;
    }

    @Override
    public void periodic() {
        if (stateDebouncer.calculate(velocityDebouncer.getAsBoolean()))
            led.setPosition(LEDState.GREEN.getColorValue());
        else
            led.setPosition(LEDState.RED.getColorValue());
    }
}
