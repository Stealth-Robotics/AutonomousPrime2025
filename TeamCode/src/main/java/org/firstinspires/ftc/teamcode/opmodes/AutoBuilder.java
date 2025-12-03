package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.AlliancePoseFlipper;
import org.firstinspires.ftc.teamcode.enums.IntakeState;
import org.firstinspires.ftc.teamcode.enums.ShooterState;
import org.firstinspires.ftc.teamcode.subsystems.RobotSystem;
import org.stealthrobotics.library.Alliance;

public class AutoBuilder {
    private final RobotSystem robot;

    //Our robot's starting positions
    public Pose CLOSE_START_POSE = new Pose();
    public Pose FAR_START_POSE = new Pose();

    //Poses that we shoot from depending on location
    private Pose CLOSE_SHOOT_POSE = new Pose();
    private Pose FAR_SHOOT_POSE = new Pose();

    //Leave pose for end of auto points
    private Pose CLOSE_LEAVE_POSE = new Pose();
    private Pose FAR_LEAVE_POSE = new Pose();

    //For intaking from loading zone
    private Pose LOADING_ZONE_POSE = new Pose();

    //Positions to start intaking the desired preload stack
    private Pose PRELOAD_1_START = new Pose();
    private Pose PRELOAD_2_START = new Pose();
    private Pose PRELOAD_3_START = new Pose();

    //Positions after intaking the preload stack
    private Pose PRELOAD_1_END = new Pose();
    private Pose PRELOAD_2_END = new Pose();
    private Pose PRELOAD_3_END = new Pose();

    //All paths needed for compiling autonomous sequences
    public Path FAR_SHOOT_TO_PRELOAD_1;
    public Path FAR_SHOOT_TO_PRELOAD_2;
    public Path FAR_SHOOT_TO_PRELOAD_3;

    public Path CLOSE_SHOOT_TO_PRELOAD_1;
    public Path CLOSE_SHOOT_TO_PRELOAD_2;
    public Path CLOSE_SHOOT_TO_PRELOAD_3;

    public Path INTAKE_PRELOAD_1;
    public Path INTAKE_PRELOAD_2;
    public Path INTAKE_PRELOAD_3;

    public Path INTAKE_PRELOAD_1_TO_FAR_SHOOT;
    public Path INTAKE_PRELOAD_2_TO_FAR_SHOOT;
    public Path INTAKE_PRELOAD_3_TO_FAR_SHOOT;

    public Path INTAKE_PRELOAD_1_TO_CLOSE_SHOOT;
    public Path INTAKE_PRELOAD_2_TO_CLOSE_SHOOT;
    public Path INTAKE_PRELOAD_3_TO_CLOSE_SHOOT;

    public Path FAR_SHOOT_TO_LEAVE;
    public Path CLOSE_SHOOT_TO_LEAVE;

    public Path FAR_SHOOT_TO_LOADING_ZONE;
    public Path LOADING_ZONE_TO_FAR_SHOOT;

    public AutoBuilder(RobotSystem robot) {
        this.robot = robot;

        if (Alliance.get() == Alliance.RED) {
            flipPoses();
        }

        buildPaths();
    }

    private void buildPaths() {

    }

    // Subsystem command macros
    private Command spinup() {
        return new InstantCommand(() -> robot.shooter.setState(ShooterState.SHOOT));
    }

    private Command intake() {
        return new InstantCommand(() -> robot.intake.setState(IntakeState.INTAKE));
    }

    private Command outtake() {
        return new InstantCommand(() -> robot.intake.setState(IntakeState.OUTTAKE));
    }

    private Command stopIntake() {
        return new InstantCommand(() -> robot.intake.setState(IntakeState.IDLE));
    }

    // Pose flipper if on red alliance
    private void flipPoses() {
        CLOSE_START_POSE = AlliancePoseFlipper.flip(CLOSE_START_POSE);
        FAR_START_POSE = AlliancePoseFlipper.flip(FAR_START_POSE);
        CLOSE_SHOOT_POSE = AlliancePoseFlipper.flip(CLOSE_SHOOT_POSE);
        FAR_SHOOT_POSE = AlliancePoseFlipper.flip(FAR_SHOOT_POSE);
        CLOSE_LEAVE_POSE = AlliancePoseFlipper.flip(CLOSE_LEAVE_POSE);
        FAR_LEAVE_POSE = AlliancePoseFlipper.flip(FAR_LEAVE_POSE);
        LOADING_ZONE_POSE = AlliancePoseFlipper.flip(LOADING_ZONE_POSE);
        PRELOAD_1_START = AlliancePoseFlipper.flip(PRELOAD_1_START);
        PRELOAD_2_START = AlliancePoseFlipper.flip(PRELOAD_2_START);
        PRELOAD_3_START = AlliancePoseFlipper.flip(PRELOAD_3_START);
        PRELOAD_1_END = AlliancePoseFlipper.flip(PRELOAD_1_END);
        PRELOAD_2_END = AlliancePoseFlipper.flip(PRELOAD_2_END);
        PRELOAD_3_END = AlliancePoseFlipper.flip(PRELOAD_3_END);
    }
}
