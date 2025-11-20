package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;

public class GamepadBindings {
    public static class DriverBindings {
        // DRIVING
        public static Button RESET_HEADING = Button.RIGHT_STICK_BUTTON;
        public static Button RESET_ROBOT_POSITION = Button.START;

        // INTAKE
        public static Trigger INTAKE = Trigger.RIGHT_TRIGGER;
        public static Trigger OUTTAKE = Trigger.LEFT_TRIGGER;
    }

    public static class OperatorBindings {
        // SPINDEXER
        public static Button EMERGENCY_RESET_SPINDEXER = Button.BACK;

        // SHOOTING
        public static Button SHOOT_PATTERN = Button.LEFT_BUMPER;
        public static Button SHOOT_RAPID = Button.RIGHT_BUMPER;

        public static Button TOGGLE_PRE_SHOOTER_SPIN_UP = Button.Y;

        // MOTIF
        public static Button SET_MOTIF_GPP = Button.DPAD_LEFT;
        public static Button SET_MOTIF_PGP = Button.DPAD_UP;
        public static Button SET_MOTIF_PPG = Button.DPAD_RIGHT;
    }
}
