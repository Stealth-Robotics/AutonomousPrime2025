package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class Teleop extends StealthOpMode {
    GamepadEx driveGamepad;
    GamepadEx operatorGamepad;

    DriveSubsystem drive;
//    ShooterSubsystem shooter;
//    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        drive = new DriveSubsystem(hardwareMap);
//        shooter = new ShooterSubsystem(hardwareMap);
//        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);

        register(drive);

        //Setup default commands
        drive.setDefaultCommand(drive.driveTeleop(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX()));

        //Configure gamepad bindings
        configureBindings();
    }

    private void configureBindings() {
        driveGamepad.getGamepadButton(GamepadBindings.DriverBindings.RESET_HEADING).whenPressed(() -> drive.resetHeading());
//        driveGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(shooter.spinToVelocity());
//        driveGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(shooter.stop());
//
//        Trigger intakeTrigger = new Trigger(() -> operatorGamepad.getTrigger(GamepadBindings.OperatorBindings.INTAKE) > 0.01);
//        intakeTrigger.whileActiveContinuous(new IntakeCommand(intake, spindexer));
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
