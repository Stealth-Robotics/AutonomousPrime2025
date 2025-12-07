package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.util.FarAuto;
import org.firstinspires.ftc.teamcode.commands.AutonomousShootCommand;
import org.firstinspires.ftc.teamcode.commands.SaveSubsystemData;
import org.firstinspires.ftc.teamcode.commands.SeeMotifCommand;

public class NothingAuto extends FarAuto {
    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedNothing", group = "Red")
    public static class RedNothing extends NothingAuto {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueNothing", group = "Blue")
    public static class BlueNothing extends NothingAuto {
    }
}
