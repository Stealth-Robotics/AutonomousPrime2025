//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import org.firstinspires.ftc.teamcode.Artifact;
//import org.firstinspires.ftc.teamcode.AutoToTeleopData;
//import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
//import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
//
//public class SaveSubsystemData extends SequentialCommandGroup {
//    public SaveSubsystemData(RobotSystem robot) {
//        addCommands(
//                new InstantCommand(() -> AutoToTeleopData.spindexerTicks = robot.spindexer.getTicks()),
//                new InstantCommand(() -> AutoToTeleopData.turretTicks = robot.turret.getRawTicks()),
//                new InstantCommand(() -> {
//                    Artifact[] artifacts = robot.spindexer.getCurrentArtifacts();
//                    AutoToTeleopData.slot1Artifact = artifacts[0];
//                    AutoToTeleopData.slot2Artifact = artifacts[1];
//                    AutoToTeleopData.slot3Artifact = artifacts[2];
//                })
//        );
//    }
//}
