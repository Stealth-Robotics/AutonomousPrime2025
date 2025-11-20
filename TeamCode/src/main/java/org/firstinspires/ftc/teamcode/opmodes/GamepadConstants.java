package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;
import com.qualcomm.robotcore.hardware.Gamepad;

public class GamepadConstants {
    public static class Rumble {
        public static final Gamepad.RumbleEffect READY_SHOOT = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)
                .build();

        public static final Gamepad.RumbleEffect ENDGAME = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 0.0, 400)
                .addStep(0.0, 1.0, 400)
                .addStep(1.0, 0.0, 400)
                .addStep(0.0, 1.0, 400)
                .addStep(1.0, 0.0, 400)
                .addStep(0.0, 1.0, 400)
                .addStep(1.0, 0.0, 400)
                .addStep(0.0, 1.0, 400)
                .build();
    }

    public static class DriverBindings {
        // DRIVING
        public static final Button RESET_HEADING = Button.RIGHT_STICK_BUTTON;
        public static final Button RESET_ROBOT_POSITION = Button.START;

        // INTAKE
        public static final Trigger INTAKE = Trigger.RIGHT_TRIGGER;
        public static final Trigger OUTTAKE = Trigger.LEFT_TRIGGER;
    }

    public static class OperatorBindings {
        // SPINDEXER
        public static final Button EMERGENCY_RESET_SPINDEXER = Button.BACK;

        // SHOOTING
        public static final Button SHOOT_PATTERN = Button.LEFT_BUMPER;
        public static final Button SHOOT_RAPID = Button.RIGHT_BUMPER;

        public static final Button TOGGLE_PRE_SHOOTER_SPIN_UP = Button.Y;

        // MOTIF
        public static final Button SET_MOTIF_GPP = Button.DPAD_LEFT;
        public static final Button SET_MOTIF_PGP = Button.DPAD_UP;
        public static final Button SET_MOTIF_PPG = Button.DPAD_RIGHT;
    }
}
