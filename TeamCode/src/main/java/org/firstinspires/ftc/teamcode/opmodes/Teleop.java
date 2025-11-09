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
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.ArtifactSource;
import org.firstinspires.ftc.teamcode.IntakeState;
import org.firstinspires.ftc.teamcode.RanAuto;
import org.firstinspires.ftc.teamcode.commands.IntakeDefaultCommand;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.Locale;

public class Teleop extends StealthOpMode {
    private GamepadEx driveGamepad;
//    private GamepadEx operatorGamepad;

    private int matchTime;
    private boolean doEndgameSignal = true;
    private boolean doReadyToShootRumble = true;

    private DriveSubsystem drive;
//    private ShooterSubsystem shooter;
    private IntakeSubsystem intake;
    private SpindexerSubsystem spindexer;
//    private TurretSubsystem turret;

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
//        operatorGamepad = new GamepadEx(gamepad2);

        drive = new DriveSubsystem(hardwareMap);
//        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap, !RanAuto.didRunAuto());
//        turret = new TurretSubsystem(hardwareMap, false);

        //Setup default commands
        intake.setDefaultCommand(new IntakeDefaultCommand(intake, () -> driveGamepad.getTrigger(GamepadBindings.DriverBindings.INTAKE) - driveGamepad.getTrigger(GamepadBindings.DriverBindings.OUTTAKE)));
        drive.setDefaultCommand(drive.driveTeleop(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX()));

        //Configure gamepad bindings
        configureBindings();

        //Setup triggers for gamepad rumble
        configureRumble();

        //Configure subsystem triggers (state transitions)
        configureTriggers();

//        telemetry.speak("Who Da Best?");
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
    }

    private void configureBindings() {
        driveGamepad.getGamepadButton(GamepadBindings.DriverBindings.RESET_HEADING).whenPressed(() -> drive.resetHeading());

        driveGamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(spindexer.rotateEmptyToIntake());
        driveGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(spindexer.rotateClosestArtifactToShoot());
        driveGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(spindexer.rotateArtifactToShoot(Artifact.GREEN));
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

    private void configureTriggers() {
        Trigger spindexerSpinEmptyToIntakeTrigger = new Trigger(() -> ((
                intake.getState() == IntakeState.INTAKE || intake.getState() == IntakeState.OUTTAKE) &&
                !spindexer.isFull()));
        spindexerSpinEmptyToIntakeTrigger.whenActive(spindexer.rotateEmptyToIntake());

        Trigger indexArtifact = new Trigger(() -> ((
                        (intake.getState() == IntakeState.INTAKE || intake.getState() == IntakeState.OUTTAKE) &&
                        !spindexer.isFull()) &&
                        spindexer.atPosition() &&
                        intake.getSensedArtifact() != Artifact.EMPTY)
        );
        indexArtifact.whenActive(new InstantCommand(() -> spindexer.updateArtifactState(intake.getSensedArtifact(), ArtifactSource.INTAKE)));

        //After intaking rotate spindexer to empty slot if not full yet
        indexArtifact.and(new Trigger(() -> !spindexer.isFull())).whenActive(spindexer.rotateEmptyToIntake());
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
    public static class RedTeleop extends Teleop {
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "Blue Teleop", group = "Blue")
    public static class BlueTeleop extends Teleop {
    }
}
