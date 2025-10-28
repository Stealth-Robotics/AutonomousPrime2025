package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.VisionSubsystem;
import org.stealthrobotics.library.Commands;
import org.stealthrobotics.library.opmodes.StealthOpMode;

@Autonomous(name = "TestAuto")
public class TestAuto extends StealthOpMode {
    VisionSubsystem vision;
    Follower follower;

    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    @Override
    public void initialize() {
        vision = new VisionSubsystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(startPose);
    }

    @Override
    public Command getAutoCommand() {
        return new SequentialCommandGroup(

        );
    }
}
