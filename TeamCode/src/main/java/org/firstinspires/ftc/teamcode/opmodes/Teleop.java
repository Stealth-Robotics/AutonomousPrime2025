package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.Locale;

import kotlin.time.Instant;

public class Teleop extends StealthOpMode {
    private GamepadEx driveGamepad;
    private GamepadEx operatorGamepad;

    private RobotSystem robot;

    private int matchTime;
    private boolean doEndgameSignal = true;
    private boolean doReadyToShootRumble = true;

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
        operatorGamepad = new GamepadEx(gamepad2);

        robot = new RobotSystem(
                hardwareMap,
                new Trigger(() -> false),
                new Trigger(() -> false),
                operatorGamepad.getGamepadButton(GamepadBindings.OperatorBindings.SHOOT_PATTERN),
                operatorGamepad.getGamepadButton(GamepadBindings.OperatorBindings.SHOOT_RAPID)
        );

        //Setup driving suppliers
//        robot.setDriverControl(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX());

        //Configure gamepad bindings
        configureBindings();

        //Setup triggers for gamepad rumble
        configureRumble();

        //Set DS telemetry to allow rich text formatting
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        //Transfer subsystem data from auto into teleop
//        LoadSubsystemData loadAutoDataIntoTeleop = new LoadSubsystemData(robot);
//        loadAutoDataIntoTeleop.schedule();
    }

    private void configureBindings() {
//        driveGamepad.getGamepadButton(GamepadBindings.DriverBindings.RESET_HEADING).whenPressed(() -> robot.drive.resetHeading());
//        driveGamepad.getGamepadButton(GamepadBindings.DriverBindings.RESET_ROBOT_POSITION).whenPressed(() -> robot.drive.resetToPosition(0, 0));

//        operatorGamepad.getGamepadButton(GamepadBindings.OperatorBindings.EMERGENCY_RESET_SPINDEXER).whenPressed(
//                new ConditionalCommand(
//                        new EmergencyResetSpindexer(robot.spindexer, robot.intake),
//                        new InstantCommand(),
//                        () -> robot.getState() == RobotSystem.RobotState.IDLE
//                )
//        );

        //Manual overrides for motif pattern
        operatorGamepad.getGamepadButton(GamepadBindings.OperatorBindings.SET_MOTIF_PPG).whenPressed(new InstantCommand(() -> Motif.setMotif(Motif.MotifType.PPG)));
        operatorGamepad.getGamepadButton(GamepadBindings.OperatorBindings.SET_MOTIF_PGP).whenPressed(new InstantCommand(() -> Motif.setMotif(Motif.MotifType.PGP)));
        operatorGamepad.getGamepadButton(GamepadBindings.OperatorBindings.SET_MOTIF_GPP).whenPressed(new InstantCommand(() -> Motif.setMotif(Motif.MotifType.GPP)));

        driveGamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(robot.setRobotState(RobotSystem.RobotState.SHOOT));
        driveGamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(robot.setRobotState(RobotSystem.RobotState.IDLE));
        driveGamepad.getGamepadButton(GamepadKeys.Button.X).whenPressed(robot.setRobotState(RobotSystem.RobotState.PRE_RAPID));
    }

    private void configureRumble() {
//        Trigger endGameBuzz = new Trigger(() -> doEndgameSignal && matchTime <= 20);
//        endGameBuzz.whenActive(new ParallelCommandGroup(
//                new InstantCommand(() -> gamepad1.runRumbleEffect(endgameRumble)),
//                new InstantCommand(() -> doEndgameSignal = false)
//        ));
//
//        Trigger readyToShootRumbleTrigger = new Trigger(() -> robot.spindexer.isFull() && doReadyToShootRumble);
//        readyToShootRumbleTrigger.whenActive(new ParallelCommandGroup(
//                new InstantCommand(() -> gamepad1.runRumbleEffect(readyToShootRumble)),
//                new InstantCommand(() -> doReadyToShootRumble = false)
//        ));
//
//        Trigger resetReadyToShootRumble = new Trigger(() -> !doReadyToShootRumble && !robot.spindexer.isFull());
//        resetReadyToShootRumble.whenActive(new InstantCommand(() -> doReadyToShootRumble = true));
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
