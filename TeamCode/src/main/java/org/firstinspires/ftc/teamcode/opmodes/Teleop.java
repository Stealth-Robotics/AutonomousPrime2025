package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.PoseTracker;
import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.ShootCommand;
import org.firstinspires.ftc.teamcode.commands.TurretDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

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

        drive = new DriveSubsystem(hardwareMap, PoseTracker.getEstimatedPose());
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);

        register(drive, shooter, intake, spindexer, turret);

        //Setup default commands
        turret.setDefaultCommand(new TurretDefaultCommand(turret, Alliance.get()));
        intake.setDefaultCommand(new IntakeDefaultCommand(intake, () -> (driveGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) - driveGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER))));
        drive.setDefaultCommand(drive.driveTeleop(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX()));

        //Configure gamepad bindings
        configureBindings();
    }

    private void configureBindings() {
        driveGamepad.getGamepadButton(GamepadBindings.DriverBindings.RESET_HEADING).whenPressed(() -> drive.resetHeading());
        driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(spindexer.toggleMode());
        driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(spindexer.controlSlot(SpindexerSubsystem.SpindexerSlot.ONE));
        driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(spindexer.controlSlot(SpindexerSubsystem.SpindexerSlot.TWO));
        driveGamepad.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(spindexer.controlSlot(SpindexerSubsystem.SpindexerSlot.THREE));

        driveGamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ShootCommand(shooter, intake));
        driveGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(turret.unWrapFully());
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
