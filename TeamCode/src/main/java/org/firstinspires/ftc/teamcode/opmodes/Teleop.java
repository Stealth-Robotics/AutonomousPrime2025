package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.EncoderSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class Teleop extends StealthOpMode {
    private GamepadEx driveGamepad;
    private GamepadEx operatorGamepad;

    EncoderSubsystem encoder;

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        encoder = new EncoderSubsystem(hardwareMap);
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "Red Teleop", group = "Red")
    public static class RedTeleop extends Teleop { }

    @SuppressWarnings("unused")
    @TeleOp(name = "Blue Teleop", group = "Blue")
    public static class BlueTeleop extends Teleop { }
}
