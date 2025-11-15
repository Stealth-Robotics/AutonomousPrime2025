package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
import java.util.function.DoubleSupplier;

public class ShootRapidCommand extends SequentialCommandGroup {
    public ShootRapidCommand(ShooterSubsystem shooter, IntakeSubsystem intake, SpindexerSubsystem spindexer,  DoubleSupplier spindexerSize) {
        try {
            int size = (int) spindexerSize.getAsDouble();
            for (int i = 0; i < size; i++) {
                boolean finalShot = (i == size - 1);
                addCommands(
                        spindexer.rotateClosestArtifactToShoot().withTimeout(1000),
                        new ShootCommand(shooter, intake, spindexer, () -> finalShot)
                );
            }
        }
        catch (Exception e) {
            throw new RuntimeException("Stupid addCommands failed in ShootRapidCommand....MARCO FIX THIS!!!");
        }
    }
}