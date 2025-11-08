package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.AlliancePoseFlipper;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.opmodes.StealthOpMode;

public class TestAuto extends StealthOpMode {
    private LimelightSubsystem limelight;
    private Follower follower;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    @Override
    public void whileWaitingToStart() {
        Motif.setMotif(limelight.getUpdatedMotif(Motif.getMotif()));
    }

    @Override
    public void initialize() {
        limelight = new LimelightSubsystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);

        //Add all poses in the autonomous here
        AlliancePoseFlipper.flip(Alliance.get(), new Pose[]{
                startPose
        });
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> limelight.setGoalTracking())
        );
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "RedTestAuto", group = "Red")
    public static class RedTestAuto extends TestAuto {
    }

    @SuppressWarnings("unused")
    @Autonomous(name = "BlueTestAuto", group = "Blue")
    public static class BlueTestAuto extends TestAuto {
    }
}
