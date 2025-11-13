package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.IntakeState;
import org.firstinspires.ftc.teamcode.PoseSupplier;
import org.firstinspires.ftc.teamcode.TurretState;
import org.firstinspires.ftc.teamcode.commands.DumbOuttakeCommand;
import org.firstinspires.ftc.teamcode.commands.EmergencyResetSpindexer;
import org.firstinspires.ftc.teamcode.commands.LoadSubsystemData;
import org.firstinspires.ftc.teamcode.commands.SmartIntakeCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.Locale;

public class Teleop extends StealthOpMode {
    private GamepadEx driveGamepad;

    private int matchTime;
    private boolean doEndgameSignal = true;
    private boolean doReadyToShootRumble = true;

    private DriveSubsystem drive;
    private IntakeSubsystem intake;
    private SpindexerSubsystem spindexer;
    private TurretSubsystem turret;
    private LimelightSubsystem limelight;

    private final ElapsedTime matchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private final Gamepad.RumbleEffect readyToShootRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 1.0, 500)
            .build();

    private final Gamepad.RumbleEffect endgameRumble = new Gamepad.RumbleEffect.Builder()
            .addStep(1.0, 0.0, 400)
            .addStep(0.0, 1.0, 400)
            .addStep(1.0, 0.0, 400)
            .addStep(0.0, 1.0, 400)
            .addStep(1.0, 0.0, 400)
            .addStep(0.0, 1.0, 400)
            .addStep(1.0, 0.0, 400)
            .addStep(0.0, 1.0, 400)
            .build();

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);

        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        //! Undo after testing
//        turret = new TurretSubsystem(hardwareMap, new PoseSupplier(() -> drive.getPoseX(), () -> drive.getPoseY(), () -> AngleUnit.RADIANS.toDegrees(drive.getHeading())));
        limelight = new LimelightSubsystem(hardwareMap);

        //Setup default commands
        drive.setDefaultCommand(drive.driveTeleop(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX()));

        //Configure gamepad bindings
        configureBindings();

        //Setup triggers for gamepad rumble
        configureRumble();

        //Set DS telemetry to allow rich text formatting
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        //Transfer subsystem data from auto into teleop
        //! Undo after testing
//        LoadSubsystemData loadAutoDataIntoTeleop = new LoadSubsystemData(drive, spindexer, turret);
//        loadAutoDataIntoTeleop.schedule();
    }

    private void configureBindings() {
        driveGamepad.getGamepadButton(GamepadBindings.DriverBindings.RESET_HEADING).whenPressed(() -> drive.resetHeading());

        //!Undo after testing
//        driveGamepad.getGamepadButton(GamepadBindings.DriverBindings.EMERGENCY_RESET_SPINDEXER).whenPressed(new EmergencyResetSpindexer(spindexer, intake));
//
        Trigger intakeTrigger = new Trigger(() -> driveGamepad.getTrigger(GamepadBindings.DriverBindings.INTAKE) > 0.01);
        intakeTrigger.whenActive(new InstantCommand(() ->  gamepad1.runRumbleEffect(readyToShootRumble)));
//
//        Trigger outtakeTrigger = new Trigger(() -> driveGamepad.getTrigger(GamepadBindings.DriverBindings.OUTTAKE) > 0.01);
//        outtakeTrigger.whenActive(new DumbOuttakeCommand(intake, () -> outtakeTrigger.negate().get()));
////
//        //Toggles turret between homing and searching for the goal
//        Trigger homeTurret = new Trigger(() -> driveGamepad.getButton(GamepadBindings.DriverBindings.HOME_AND_UNHOME_TURRET));
//        homeTurret.toggleWhenActive(new InstantCommand(() -> turret.setState(TurretState.HOME)), new InstantCommand(() -> turret.setState(TurretState.SEARCH)));
    }

    private void configureRumble() {
        Trigger endGameBuzz = new Trigger(() -> doEndgameSignal && matchTime <= 20);
        endGameBuzz.whenActive(new ParallelCommandGroup(
                new InstantCommand(() -> gamepad1.runRumbleEffect(endgameRumble)),
                new InstantCommand(() -> doEndgameSignal = false)
        ));

        Trigger readyToShootRumbleTrigger = new Trigger(() -> spindexer.isFull() && doReadyToShootRumble);
        readyToShootRumbleTrigger.whenActive(new ParallelCommandGroup(
                new InstantCommand(() -> gamepad1.runRumbleEffect(readyToShootRumble)),
                new InstantCommand(() -> doReadyToShootRumble = false)
        ));

        Trigger resetReadyToShootRumble = new Trigger(() -> !doReadyToShootRumble && !spindexer.isFull());
        resetReadyToShootRumble.whenActive(new InstantCommand(() -> doReadyToShootRumble = true));
    }

    @Override
    public Command getAutoCommand() {
        return new InstantCommand(matchTimer::reset);
    }

    @Override
    public void printTelemetry() {
        matchTime = (int) Math.max(0, 120 - matchTimer.time());
        String timeStr = String.format(Locale.US, "%02d", (matchTime / 60)) + ":" + String.format(Locale.US, "%02d", (matchTime - ((matchTime / 60) * 60)));
        telemetry.addLine("<h4>MATCH TIME: " + timeStr + "</h4>");
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "Red Teleop", group = "Red")
    public static class RedTeleop extends Teleop { }

    @SuppressWarnings("unused")
    @TeleOp(name = "Blue Teleop", group = "Blue")
    public static class BlueTeleop extends Teleop { }
}
