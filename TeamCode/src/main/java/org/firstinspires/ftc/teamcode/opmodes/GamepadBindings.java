package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.gamepad.GamepadKeys.*;

public class GamepadBindings {
    public static class DriverBindings {
        // DRIVING
        public static Button RESET_HEADING = Button.RIGHT_STICK_BUTTON;

        // INTAKE
        public static Trigger INTAKE = Trigger.RIGHT_TRIGGER;
        public static Trigger OUTTAKE = Trigger.LEFT_TRIGGER;

        // TURRET
        public static Button HOME_AND_UNHOME_TURRET = Button.A;

        // SPINDEXER
        public static Button EMERGENCY_RESET_SPINDEXER = Button.BACK;
    }

    public static class OperatorBindings {
    }
}
