package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class Teleop extends StealthOpMode {
    GamepadEx driveGamepad;
    GamepadEx operatorGamepad;

    DriveSubsystem drive;

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        //Get auto to teleop heading to work

        register(drive);
        drive.setDefaultCommand(drive.driveTeleop(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX()));
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "Red Teleop", group = "Red")
    public static class RedTeleop extends Teleop {
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "Blue Teleop", group = "Blue")
    public static class BlueTeleop extends Teleop {
    }
}
