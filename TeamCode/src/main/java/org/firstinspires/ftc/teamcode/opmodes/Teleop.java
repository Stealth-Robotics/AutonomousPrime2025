package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class Teleop extends StealthOpMode {
    GamepadEx driveGamepad;
    GamepadEx operatorGamepad;

    DriveSubsystem drive;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    SpindexerSubsystem spindexer;

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);

        register(drive, shooter, intake, spindexer);

        //Setup default commands
        intake.setDefaultCommand(new IntakeDefaultCommand(intake, () -> (driveGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driveGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))));
        drive.setDefaultCommand(drive.driveTeleop(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX()));

        //Configure gamepad bindings
        configureBindings();
    }

    private void configureBindings() {
        driveGamepad.getGamepadButton(GamepadBindings.DriverBindings.RESET_HEADING).whenPressed(() -> drive.resetHeading());
//        driveGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(intake.deployLoader());
//        driveGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(intake.retractLoader());
//        driveGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(intake.start());
//        driveGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(intake.stop());

//        driveGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(spindexer.rotateEmptyToIntake());
//        driveGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(spindexer.rotateEmptyToShooter());
//        driveGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(spindexer.rotateClosestArtifactToShoot());

//        operatorGamepad.getGamepadButton(GamepadBindings.OperatorBindings.INTAKE_FROM_SHOOTER).whenPressed(new IntakeFromShooterCommand())
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
