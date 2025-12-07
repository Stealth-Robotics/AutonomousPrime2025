package org.firstinspires.ftc.teamcode.autonomous.util;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.AlliancePoseFlipper;
import org.firstinspires.ftc.teamcode.enums.ShooterState;
import org.firstinspires.ftc.teamcode.systems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.stealthrobotics.library.Alliance;

public class AutoBuilder {
    private final RobotSystem robot;
    private final FollowerSubsystem follower;

    //Our robot's starting positions
    public Pose CLOSE_START_POSE = new Pose(21.2, 120, Math.toRadians(0));
    public Pose FAR_START_POSE = new Pose(57.38, 8.44, Math.toRadians(0));

    //Poses that we shoot from depending on location
    private Pose CLOSE_SHOOT_POSE = new Pose(39.166, 104.599,Math.toRadians(0));
    private Pose FAR_SHOOT_POSE = new Pose(58.16,19.46, Math.toRadians(0));

    //Leave pose for end of auto points
    private Pose CLOSE_LEAVE_POSE = new Pose(48.54, 72,Math.toRadians(0));
    private Pose FAR_LEAVE_POSE = new Pose(47,23, Math.toRadians(0));
    private Pose RAMP_PARK_POSE = new Pose(20.471,69.988,Math.toRadians(0));

    private Pose FAR_LEAVE_FLAT_POSE = new Pose(20,18, Math.toRadians(0));

    //For intaking from loading zone
    private Pose LOADING_ZONE_POSE = new Pose(14,11);

    //Positions to start intaking the desired preload stack
    private Pose PRESET_1_START = new Pose(50,84, Math.toRadians(0));
    private Pose PRESET_2_START = new Pose(46,60, Math.toRadians(0));
    private Pose PRESET_3_START = new Pose(46,34, Math.toRadians(0));

    private Pose PRESET_1_MIDDLE_1 = new Pose(37.4,84, Math.toRadians(0));
    private Pose PRESET_1_MIDDLE_2 = new Pose(31.9,84, Math.toRadians(0));

    private Pose PRESET_2_MIDDLE_1 = new Pose(37.4,60, Math.toRadians(0));
    private Pose PRESET_2_MIDDLE_2 = new Pose(31.9,60, Math.toRadians(0));

    private Pose PRESET_3_MIDDLE_1 = new Pose(37.4,34, Math.toRadians(0));
    private Pose PRESET_3_MIDDLE_2 = new Pose(31.9,34, Math.toRadians(0));

    //Positions after intaking the preload stack
    private Pose PRESET_1_END = new Pose(15,84, Math.toRadians(0));
    private Pose PRESET_2_END = new Pose(19,60, Math.toRadians(0));
    private Pose PRESET_3_END = new Pose(10,34, Math.toRadians(0));

    //All paths needed for compiling autonomous sequences
    public PathChain FAR_START_TO_FAR_SHOOT;
    public PathChain NEAR_START_TO_NEAR_SHOOT;

    public PathChain FAR_SHOOT_FLAT_LEAVE;

    public PathChain FAR_SHOOT_TO_PRESET_1;
    public PathChain FAR_SHOOT_TO_PRESET_2;
    public PathChain FAR_SHOOT_TO_PRESET_3;

    public PathChain CLOSE_SHOOT_TO_PRESET_1;
    public PathChain CLOSE_SHOOT_TO_PRESET_2;
    public PathChain CLOSE_SHOOT_TO_PRESET_3;

    public PathChain INTAKE_PRESET_1_BALL_1;
    public PathChain INTAKE_PRESET_1_BALL_2;
    public PathChain INTAKE_PRESET_1_BALL_3;

    public PathChain INTAKE_PRESET_2_BALL_1;
    public PathChain INTAKE_PRESET_2_BALL_2;
    public PathChain INTAKE_PRESET_2_BALL_3;

    public PathChain INTAKE_PRESET_3_BALL_1;
    public PathChain INTAKE_PRESET_3_BALL_2;
    public PathChain INTAKE_PRESET_3_BALL_3;

    public PathChain INTAKE_PRESET_1_TO_FAR_SHOOT;
    public PathChain INTAKE_PRESET_2_TO_FAR_SHOOT;
    public PathChain INTAKE_PRESET_3_TO_FAR_SHOOT;

    public PathChain INTAKE_PRESET_1_TO_CLOSE_SHOOT;
    public PathChain INTAKE_PRESET_2_TO_CLOSE_SHOOT;
    public PathChain INTAKE_PRESET_3_TO_CLOSE_SHOOT;

    public PathChain FAR_SHOOT_TO_LEAVE;
    public PathChain CLOSE_SHOOT_TO_LEAVE;

    public PathChain START_TO_STEAL;

    public enum PresetLocation {
        NEAR,
        MIDDLE,
        FAR
    }

    public enum AutoType {
        NEAR,
        FAR
    }

    public AutoBuilder(RobotSystem robot, FollowerSubsystem followerSubsystem) {
        this.robot = robot;
        this.follower = followerSubsystem;

        if (Alliance.get() == Alliance.RED) {
            flipPoses();
        }

        buildPaths();
    }

    private void buildPaths() {
        FAR_START_TO_FAR_SHOOT = pathLinearHeading(FAR_START_POSE, FAR_SHOOT_POSE);
        NEAR_START_TO_NEAR_SHOOT = pathLinearHeading(CLOSE_START_POSE, CLOSE_SHOOT_POSE);

        INTAKE_PRESET_1_BALL_1 = pathLinearHeading(PRESET_1_START, PRESET_1_MIDDLE_1);
        INTAKE_PRESET_1_BALL_2 = pathLinearHeading(PRESET_1_MIDDLE_1, PRESET_1_MIDDLE_2);
        INTAKE_PRESET_1_BALL_3 = pathLinearHeading(PRESET_1_MIDDLE_2, PRESET_1_END);

        INTAKE_PRESET_2_BALL_1 = pathLinearHeading(PRESET_2_START, PRESET_2_MIDDLE_1);
        INTAKE_PRESET_2_BALL_2 = pathLinearHeading(PRESET_2_MIDDLE_1, PRESET_2_MIDDLE_2);
        INTAKE_PRESET_2_BALL_3 = pathLinearHeading(PRESET_2_MIDDLE_2, PRESET_2_END);

        INTAKE_PRESET_3_BALL_1 = pathLinearHeading(PRESET_3_START, PRESET_3_MIDDLE_1);
        INTAKE_PRESET_3_BALL_2 = pathLinearHeading(PRESET_3_MIDDLE_1, PRESET_3_MIDDLE_2);
        INTAKE_PRESET_3_BALL_3 = pathLinearHeading(PRESET_3_MIDDLE_2, PRESET_3_END);

        FAR_SHOOT_FLAT_LEAVE = pathLinearHeading(FAR_SHOOT_POSE, FAR_LEAVE_FLAT_POSE);

        FAR_SHOOT_TO_PRESET_1 = pathLinearHeading(FAR_SHOOT_POSE, PRESET_1_START);
        FAR_SHOOT_TO_PRESET_2 = pathLinearHeading(FAR_SHOOT_POSE, PRESET_2_START);
        FAR_SHOOT_TO_PRESET_3 = pathLinearHeading(FAR_SHOOT_POSE, PRESET_3_START);
        CLOSE_SHOOT_TO_PRESET_1 = pathLinearHeading(CLOSE_SHOOT_POSE, PRESET_1_START);
        CLOSE_SHOOT_TO_PRESET_2 = pathLinearHeading(CLOSE_SHOOT_POSE, PRESET_2_START);
        CLOSE_SHOOT_TO_PRESET_3 = pathLinearHeading(CLOSE_SHOOT_POSE, PRESET_3_START);
        INTAKE_PRESET_1_TO_FAR_SHOOT = pathLinearHeading(PRESET_1_END, FAR_SHOOT_POSE);
        INTAKE_PRESET_2_TO_FAR_SHOOT = pathLinearHeading(PRESET_2_END, FAR_SHOOT_POSE);
        INTAKE_PRESET_3_TO_FAR_SHOOT = pathLinearHeading(PRESET_3_END, FAR_SHOOT_POSE);
        INTAKE_PRESET_1_TO_CLOSE_SHOOT = pathLinearHeading(PRESET_1_END, CLOSE_SHOOT_POSE);
        INTAKE_PRESET_2_TO_CLOSE_SHOOT = pathLinearHeading(PRESET_2_END, CLOSE_SHOOT_POSE);
        INTAKE_PRESET_3_TO_CLOSE_SHOOT = pathLinearHeading(PRESET_3_END, CLOSE_SHOOT_POSE);
        FAR_SHOOT_TO_LEAVE = pathLinearHeading(FAR_SHOOT_POSE, FAR_LEAVE_POSE);
        CLOSE_SHOOT_TO_LEAVE = pathLinearHeading(CLOSE_SHOOT_POSE, CLOSE_LEAVE_POSE);
        START_TO_STEAL = follower.pathBuilder().addPath(new BezierLine(FAR_START_POSE, LOADING_ZONE_POSE))
                .setLinearHeadingInterpolation(0,0)
                .build();
    }

    //method for easy path creation
    private PathChain pathLinearHeading(Pose pose1, Pose pose2) {
        return (follower.pathBuilder().addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(),pose2.getHeading())
                .build());
    }

    public PathChain farShootToLoadingZone(double startHeading, double endHeading) {
        return (follower.pathBuilder().addPath(new BezierLine(FAR_SHOOT_POSE, LOADING_ZONE_POSE))
                .setLinearHeadingInterpolation(startHeading, endHeading))
                .build();
    }

    public PathChain loadingZoneToFarShoot(double startHeading, double endHeading) {
        return (follower.pathBuilder().addPath(new BezierLine(LOADING_ZONE_POSE, FAR_SHOOT_POSE))
                .setLinearHeadingInterpolation(startHeading, endHeading))
                .build();
    }

    public PathChain getPresetPathBall1(PresetLocation preset) {
        switch (preset) {
            case NEAR:
                return INTAKE_PRESET_1_BALL_1;
            case MIDDLE:
                return INTAKE_PRESET_2_BALL_1;
            default:
                return INTAKE_PRESET_3_BALL_1;
        }
    }

    public PathChain getPresetPathBall2(PresetLocation preset) {
        switch (preset) {
            case NEAR:
                return INTAKE_PRESET_1_BALL_2;
            case MIDDLE:
                return INTAKE_PRESET_2_BALL_2;
            default:
                return INTAKE_PRESET_3_BALL_2;
        }
    }

    public PathChain getPresetPathBall3(PresetLocation preset) {
        switch (preset) {
            case NEAR:
                return INTAKE_PRESET_1_BALL_3;
            case MIDDLE:
                return INTAKE_PRESET_2_BALL_3;
            default:
                return INTAKE_PRESET_3_BALL_3;
        }
    }
    
    
    //Intake the specified preset stack of artifacts
    private Command intakePreset(PresetLocation preset) {
        return new SequentialCommandGroup(
                intake(),
                follower.followPath(getPresetPathBall1(preset), 0.4, true),
                new WaitCommand(250),
                follower.followPath(getPresetPathBall2(preset), 0.4, true),
                new WaitCommand(250),
                follower.followPath(getPresetPathBall3(preset), 0.4, true),
                spinup(),
                new WaitCommand(300),
                idle()
        );
    }

    private Command nearToPreset(PresetLocation preset) {
        PathChain selectPath = (preset == PresetLocation.NEAR ? CLOSE_SHOOT_TO_PRESET_1 : preset == PresetLocation.MIDDLE ? CLOSE_SHOOT_TO_PRESET_2 : CLOSE_SHOOT_TO_PRESET_3);
        return follower.followPath(selectPath, 0.75, false);
    }
    private Command farToPreset(PresetLocation preset) {
        PathChain selectPath = (preset == PresetLocation.NEAR ? FAR_SHOOT_TO_PRESET_1 : preset == PresetLocation.MIDDLE ? FAR_SHOOT_TO_PRESET_2 : FAR_SHOOT_TO_PRESET_3);
        return follower.followPath(selectPath, 0.75, false);
    }
    private Command presetToNear(PresetLocation preset) {
        PathChain selectPath = (preset == PresetLocation.NEAR ? INTAKE_PRESET_1_TO_CLOSE_SHOOT : preset == PresetLocation.MIDDLE ? INTAKE_PRESET_2_TO_CLOSE_SHOOT : INTAKE_PRESET_3_TO_CLOSE_SHOOT);
        return follower.followPath(selectPath, 0.6, true);
    }
    private Command presetToFar(PresetLocation preset) {
        PathChain selectPath = (preset == PresetLocation.NEAR ? INTAKE_PRESET_1_TO_FAR_SHOOT : preset == PresetLocation.MIDDLE ? INTAKE_PRESET_2_TO_FAR_SHOOT: INTAKE_PRESET_3_TO_FAR_SHOOT);
        return follower.followPath(selectPath, 0.6, true);
    }

    public Command farFlatLeave() {
        return follower.followPath(FAR_SHOOT_FLAT_LEAVE, 0.75, true);
    }

    public Command fromStartToShootNear() {
        return new SequentialCommandGroup(
                follower.followPath(NEAR_START_TO_NEAR_SHOOT, true)
        );
    }

    public Command fromStartToShootFar() {
        return new SequentialCommandGroup(
                follower.followPath(FAR_START_TO_FAR_SHOOT, true)
        );
    }
    
    public Command cycle(PresetLocation preset, AutoType autoType) {
        return new SequentialCommandGroup(
                new ConditionalCommand(nearToPreset(preset), farToPreset(preset), () -> autoType == AutoType.NEAR),
                intakePreset(preset),
                new ConditionalCommand(presetToNear(preset), presetToFar(preset), () -> autoType == AutoType.NEAR)
        );
    }

    public Command halfCycle(PresetLocation preset, AutoType autoType) {
        return new SequentialCommandGroup(
                new ConditionalCommand(nearToPreset(preset), farToPreset(preset), () -> autoType == AutoType.NEAR),
                intakePreset(preset)
        );
    }

    public Command stealCycle() {
        int sign = (Alliance.get() == Alliance.RED ? -1 : 1); //Flip angles on opposite alliance
        return new SequentialCommandGroup(
                intake(),
                follower.followPath(START_TO_STEAL,false),
                follower.followPath(loadingZoneToFarShoot(Math.toRadians(0), Math.toRadians(0)), false),
                follower.followPath(farShootToLoadingZone(Math.toRadians(0), Math.toRadians(sign * 10)), false),
                follower.followPath(loadingZoneToFarShoot(Math.toRadians(sign * 10), Math.toRadians(sign * 10)), false),
                follower.followPath(farShootToLoadingZone(Math.toRadians(sign * 10), Math.toRadians(sign * -10)), false),
                follower.followPath(loadingZoneToFarShoot(Math.toRadians(sign * -10), Math.toRadians(sign * -10)), false),
                follower.followPath(farShootToLoadingZone(Math.toRadians(sign * -10), Math.toRadians(0)), true)
        );
    }

    //park commands
    public Command parkNear() {
        return (follower.followPath(CLOSE_SHOOT_TO_LEAVE, 0.75, true));
    }
    public Command parkFar() {
        return (follower.followPath(FAR_SHOOT_TO_LEAVE, 0.75, true));
    }


    // Subsystem command macros
    private Command spinup() {
        return robot.shooter.setState(ShooterState.SHOOT);
    }

    private Command intake() {
        return robot.setRobotState(RobotSystem.RobotState.INTAKE);
    }

    private Command outtake() {
        return robot.setRobotState(RobotSystem.RobotState.OUTTAKE);
    }

    private Command idle() {
        return robot.setRobotState(RobotSystem.RobotState.IDLE);
    }

    // Pose flipper if on red alliance
    private void flipPoses() {
        CLOSE_START_POSE = AlliancePoseFlipper.flip(CLOSE_START_POSE);
        FAR_START_POSE = AlliancePoseFlipper.flip(FAR_START_POSE);

        CLOSE_SHOOT_POSE = AlliancePoseFlipper.flip(CLOSE_SHOOT_POSE);
        FAR_SHOOT_POSE = AlliancePoseFlipper.flip(FAR_SHOOT_POSE);

        CLOSE_LEAVE_POSE = AlliancePoseFlipper.flip(CLOSE_LEAVE_POSE);
        FAR_LEAVE_POSE = AlliancePoseFlipper.flip(FAR_LEAVE_POSE);
        RAMP_PARK_POSE = AlliancePoseFlipper.flip(RAMP_PARK_POSE);

        LOADING_ZONE_POSE = AlliancePoseFlipper.flip(LOADING_ZONE_POSE);

        PRESET_1_START = AlliancePoseFlipper.flip(PRESET_1_START);
        PRESET_2_START = AlliancePoseFlipper.flip(PRESET_2_START);
        PRESET_3_START = AlliancePoseFlipper.flip(PRESET_3_START);

        PRESET_1_MIDDLE_1 = AlliancePoseFlipper.flip(PRESET_1_MIDDLE_1);
        PRESET_1_MIDDLE_2 = AlliancePoseFlipper.flip(PRESET_1_MIDDLE_2);
        PRESET_2_MIDDLE_1 = AlliancePoseFlipper.flip(PRESET_2_MIDDLE_1);
        PRESET_2_MIDDLE_2 = AlliancePoseFlipper.flip(PRESET_2_MIDDLE_2);
        PRESET_3_MIDDLE_1 = AlliancePoseFlipper.flip(PRESET_3_MIDDLE_1);
        PRESET_3_MIDDLE_2 = AlliancePoseFlipper.flip(PRESET_3_MIDDLE_2);

        FAR_LEAVE_FLAT_POSE = AlliancePoseFlipper.flip(FAR_LEAVE_FLAT_POSE);

        PRESET_1_END = AlliancePoseFlipper.flip(PRESET_1_END);
        PRESET_2_END = AlliancePoseFlipper.flip(PRESET_2_END);
        PRESET_3_END = AlliancePoseFlipper.flip(PRESET_3_END);
    }
}
