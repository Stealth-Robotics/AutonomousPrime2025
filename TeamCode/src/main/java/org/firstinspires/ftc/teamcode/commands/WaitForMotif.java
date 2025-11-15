package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.Motif;

public class WaitForMotif extends SequentialCommandGroup {
    public WaitForMotif() {
        addCommands(
                new WaitUntilCommand(() -> Motif.getMotif() != Motif.MotifType.NULL).withTimeout(2000)
        );
    }
}
