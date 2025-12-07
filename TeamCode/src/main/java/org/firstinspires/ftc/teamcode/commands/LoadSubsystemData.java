package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.AutoToTeleopData;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.stealthrobotics.library.Alliance;

public class LoadSubsystemData extends SequentialCommandGroup {
    public LoadSubsystemData(RobotSystem robot) {
        addCommands(
                new InstantCommand(() -> {
                    robot.spindexer.setEncoderOffset(AutoToTeleopData.spindexerTicks);
                    robot.turret.setEncoderOffset(AutoToTeleopData.turretTicks);
                    robot.spindexer.setArtifactsInSpindexerManually(AutoToTeleopData.slot1Artifact, AutoToTeleopData.slot2Artifact, AutoToTeleopData.slot3Artifact);
                    robot.drive.setPose(AutoToTeleopData.endOfAutoPose);
                    robot.drive.setAllianceSpecificHeading(Alliance.get());
                })
        );
    }
}
