package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class Teleop extends StealthOpMode {
    GamepadEx driveGamepad;
    GamepadEx operatorGamepad;

//    DriveSubsystem drive;
    SpindexerSubsystem spindexer;
//    ShooterSubsystem shooter;

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        spindexer = new SpindexerSubsystem(hardwareMap);

//        shooter = new ShooterSubsystem(hardwareMap);

        //Get auto to teleop heading to work (so i dont need to reset odometry in the match)

//        register(shooter);
        driveGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(spindexer.loadShooter(0));
//        shooter.setDefaultCommand(new ShooterDefaultCommand(shooter, () -> driveGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)));
//        drive.setDefaultCommand(drive.driveTeleop(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX()));

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
