package org.firstinspires.ftc.teamcode.teleop;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.AutoToTeleopData;
import org.firstinspires.ftc.teamcode.commands.EmergencyResetSpindexer;
import org.firstinspires.ftc.teamcode.commands.LoadSubsystemData;
import org.firstinspires.ftc.teamcode.enums.TurretState;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

import java.util.ArrayList;
import java.util.Locale;

public class Teleop extends StealthOpMode {
    private GamepadEx driveGamepad;
    private GamepadEx operatorGamepad;

    private RobotSystem robot;

    ArrayList<String> offsets = new ArrayList<>();

    private int matchTime;
    private boolean doEndgameSignal = true;

    private final ElapsedTime matchTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    @Override
    public void initialize() {
        driveGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);

        robot = new RobotSystem(
                hardwareMap,
                new Trigger(() -> driveGamepad.getTrigger(GamepadConstants.DriverBindings.INTAKE) > 0.01),
                new Trigger(() -> driveGamepad.getTrigger(GamepadConstants.DriverBindings.OUTTAKE) > 0.01),
                operatorGamepad.getGamepadButton(GamepadConstants.OperatorBindings.SHOOT_PATTERN),
                operatorGamepad.getGamepadButton(GamepadConstants.OperatorBindings.SHOOT_RAPID)
        );

        //Setup driving suppliers
        robot.setDriverControl(() -> driveGamepad.getLeftX(), () -> driveGamepad.getLeftY(), () -> driveGamepad.getRightX());

        //Configure gamepad bindings
        configureBindings();

        //Setup triggers for gamepad rumble
        configureRumble();

        //Set DS telemetry to allow rich text formatting
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
    }

    private void configureBindings() {
        driveGamepad.getGamepadButton(GamepadConstants.DriverBindings.RESET_HEADING).whenPressed(() -> robot.drive.resetHeading());
        driveGamepad.getGamepadButton(GamepadConstants.DriverBindings.RESET_ROBOT_POSITION).whenPressed(() -> robot.drive.resetToPosition(72,72, 90));

        driveGamepad.getGamepadButton(GamepadConstants.OperatorBindings.FREEZE_TURRET_TOGGLE).toggleWhenActive(
                robot.turret.setState(TurretState.IDLE),
                new InstantCommand(() -> robot.turret.switchToOdometryControl())
        );

        operatorGamepad.getGamepadButton(GamepadConstants.OperatorBindings.BUDGE_SPINDEXER_LEFT).whenPressed(() -> robot.spindexer.moveSpindexerManually(200));
        operatorGamepad.getGamepadButton(GamepadConstants.OperatorBindings.BUDGE_SPINDEXER_RIGHT).whenPressed(() -> robot.spindexer.moveSpindexerManually(-200));

        driveGamepad.getGamepadButton(GamepadConstants.OperatorBindings.EMERGENCY_RESET_SPINDEXER).whenPressed(
                new ConditionalCommand(
                        new EmergencyResetSpindexer(robot.spindexer, robot.intake),
                        new InstantCommand(),
                        () -> robot.getState() == RobotSystem.RobotState.IDLE
                )
        );

        operatorGamepad.getGamepadButton(GamepadConstants.OperatorBindings.RESET_STATE_MACHINE).whenPressed(robot.forceIdle());

        //Sets the starting artifact for the motif (because sometimes we only need 1 or 2 to complete)
        operatorGamepad.getGamepadButton(GamepadConstants.OperatorBindings.SET_PATTERN_MODE_1).whenPressed(robot.setPatternOffset(0));
        operatorGamepad.getGamepadButton(GamepadConstants.OperatorBindings.SET_PATTERN_MODE_2).whenPressed(robot.setPatternOffset(1));
        operatorGamepad.getGamepadButton(GamepadConstants.OperatorBindings.SET_PATTERN_MODE_3).whenPressed(robot.setPatternOffset(2));
    }

    private void configureRumble() {
        Trigger endGameBuzz = new Trigger(() -> doEndgameSignal && matchTime <= 20);
        endGameBuzz.whenActive(new ParallelCommandGroup(
                new InstantCommand(() -> gamepad1.runRumbleEffect(GamepadConstants.Rumble.ENDGAME)),
                new InstantCommand(() -> doEndgameSignal = false)
        ));

        Trigger intakedArtifactTrigger = new Trigger(() -> robot.justIntaked);
        intakedArtifactTrigger.whenActive(new ParallelCommandGroup(
                new InstantCommand(() -> gamepad1.runRumbleEffect(GamepadConstants.Rumble.INTAKED)),
                new InstantCommand(() -> robot.justIntaked = false)
        ));
    }

    @Override
    public Command getAutoCommand() {
        return new InstantCommand(matchTimer::reset);
    }

    @Override
    public void printTelemetry() {
        matchTime = (int) Math.max(0, 120 - matchTimer.time());
    }

    private String matchTimerString() {
        String timeStr = String.format(Locale.US, "%02d", (matchTime / 60)) + ":" + String.format(Locale.US, "%02d", (matchTime - ((matchTime / 60) * 60)));
        return "<h4>MATCH TIME: " + timeStr + "</h4>";
    }

    @SuppressWarnings("unused")
    @TeleOp(name = "Red Teleop", group = "Red")
    public static class RedTeleop extends Teleop { }

    @SuppressWarnings("unused")
    @TeleOp(name = "Blue Teleop", group = "Blue")
    public static class BlueTeleop extends Teleop { }
}
