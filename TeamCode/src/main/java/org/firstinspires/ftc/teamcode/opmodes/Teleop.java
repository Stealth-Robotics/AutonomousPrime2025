package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.IndexerDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TurretDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.function.DoubleSupplier;

public class Teleop extends StealthOpMode {
    GamepadEx driveGamepad;
    GamepadEx operatorGamepad;

    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;
    TurretSubsystem turret;

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        register(drive);

        //Setup default commands
        DoubleSupplier intakeSupplier = () -> ((operatorGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)) - (operatorGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));
        spindexer.setDefaultCommand(new IndexerDefaultCommand(spindexer, intakeSupplier));
        turret.setDefaultCommand(new TurretDefaultCommand(turret, Alliance.get(), new Trigger(() -> operatorGamepad.getButton(GamepadKeys.Button.B))));
        intake.setDefaultCommand(new IntakeDefaultCommand(intake, () -> (driveGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driveGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))));
        drive.setDefaultCommand(drive.driveTeleop(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX()));

        //Configure gamepad bindings
        configureBindings();
    }

    private void configureBindings() {
        driveGamepad.getGamepadButton(GamepadBindings.DriverBindings.RESET_HEADING).whenPressed(() -> drive.resetHeading());
        driveGamepad.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(() -> drive.resetPosition());



        driveGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ShootCommand(shooter, intake));
        driveGamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(shooter.toggleSpinning());
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
