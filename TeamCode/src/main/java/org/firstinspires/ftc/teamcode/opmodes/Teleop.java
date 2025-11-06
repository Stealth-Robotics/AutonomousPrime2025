package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class Teleop extends StealthOpMode {
    private GamepadEx driveGamepad;
    private GamepadEx operatorGamepad;

    private DriveSubsystem drive;
    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private SpindexerSubsystem spindexer;

    private final Gamepad.RumbleEffect fullSpindexerRumbleEffect = new Gamepad.RumbleEffect.Builder()
            .addStep(0.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
            .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .addStep(0.0, 0.0, 250)  //  Pause for 250 mSec
            .addStep(1.0, 0.0, 250)  //  Rumble left motor 100% for 250 mSec
            .build();

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        drive = new DriveSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);

        register(drive, shooter, intake, spindexer);

        driveGamepad.gamepad.runRumbleEffect(fullSpindexerRumbleEffect);
        driveGamepad.gamepad.runLedEffect(new Gamepad.LedEffect.Builder().addStep(1, 0, 0, 1000).build());

        //Setup default commands
        intake.setDefaultCommand(new IntakeDefaultCommand(intake, () -> (driveGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driveGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))));
        drive.setDefaultCommand(drive.driveTeleop(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX()));

        //Configure gamepad bindings
        configureBindings();
    }

    private void configureBindings() {
        driveGamepad.getGamepadButton(GamepadBindings.DriverBindings.RESET_HEADING).whenPressed(() -> drive.resetHeading());
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
