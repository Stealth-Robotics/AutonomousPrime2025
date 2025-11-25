//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.enums.IntakeState;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//
//import java.util.function.BooleanSupplier;
//
//public class OuttakeCommand extends SequentialCommandGroup {
//    public OuttakeCommand(IntakeSubsystem intake, BooleanSupplier stop) {
//        addCommands(
//                new InstantCommand(() -> intake.setState(IntakeState.OUTTAKE)),
//                new WaitUntilCommand(stop),
//                new InstantCommand(() -> intake.setState(IntakeState.IDLE))
//        );
//    }
//}
