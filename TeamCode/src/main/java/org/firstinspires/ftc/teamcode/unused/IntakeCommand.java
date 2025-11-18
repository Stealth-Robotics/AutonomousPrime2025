//package org.firstinspires.ftc.teamcode.commands;
//
//import com.arcrobotics.ftclib.command.InstantCommand;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.arcrobotics.ftclib.command.WaitUntilCommand;
//
//import org.firstinspires.ftc.teamcode.IntakeState;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
//
//import java.util.function.BooleanSupplier;
//
//public class IntakeCommand extends SequentialCommandGroup {
//    /** Will intake and reschedule command if there is more space in spindexer.
//     * You must provide a boolean supplier that stops the process (aka a timeout in auto, or a trigger release in teleop).
//     */
//
//    public IntakeCommand(IntakeSubsystem intake, SpindexerSubsystem spindexer, BooleanSupplier stop) {
//        addCommands(
//                new InstantCommand(() -> intake.setState(IntakeState.INTAKE)),
//                new WaitUntilCommand(stop),
//                new InstantCommand(() -> intake.setState(IntakeState.IDLE))
//        );
//    }
//}
