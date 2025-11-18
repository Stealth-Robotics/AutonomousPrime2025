package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LEDState;
import org.stealthrobotics.library.StealthSubsystem;

public class LEDSubsystem extends StealthSubsystem {
    private final Servo led;

    public LEDSubsystem(HardwareMap hardwareMap) {
        led = hardwareMap.get(Servo.class, "led");
    }

    public void setState(LEDState state) {
        led.setPosition(state.getColorValue());
    }
}
