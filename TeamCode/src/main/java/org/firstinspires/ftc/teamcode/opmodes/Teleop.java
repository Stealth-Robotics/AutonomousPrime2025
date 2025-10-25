package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.GamepadBindings;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.commands.ShooterDefaultCommand;
import org.firstinspires.ftc.teamcode.commands.TurretDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.AutoToTeleStorage;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class Teleop extends StealthOpMode {
    GamepadEx driveGamepad;
    GamepadEx operatorGamepad;

//    DriveSubsystem drive;
//    SpindexerSubsystem spindexer;
//    TurretSubsystem turret;
//    ShooterSubsystem shooter;
    VisionSubsystem vision;

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

//        drive = new DriveSubsystem(hardwareMap);
//        spindexer = new SpindexerSubsystem(hardwareMap);
//        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
//        shooter = new ShooterSubsystem(hardwareMap);

//        register(shooter);

        //Transfer heading from auto to teleop
//        drive.setHeading(AutoToTeleStorage.finalAutoHeading);

        //Setup default commands
//        drive.setDefaultCommand(drive.driveTeleop(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX()));
//        turret.setDefaultCommand(new TurretDefaultCommand(turret, () -> driveGamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER), () -> driveGamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)));

        //Configure gamepad bindings
        configureBindings();
    }

    private void configureBindings() {
//        driveGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> spindexer.rotateEmptyToIntake());
//        driveGamepad.getGamepadButton(GamepadBindings.RESET_HEADING).whenPressed(() -> drive.resetHeading());
//        driveGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(shooter.spinUp(1.0));
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
