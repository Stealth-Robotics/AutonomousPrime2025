package org.firstinspires.ftc.teamcode.autonomous;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.AlliancePoseFlipper;
import org.firstinspires.ftc.teamcode.enums.IntakeState;
import org.firstinspires.ftc.teamcode.enums.ShooterState;
import org.firstinspires.ftc.teamcode.systems.FollowerSubsystem;
import org.firstinspires.ftc.teamcode.systems.RobotSystem;
import org.stealthrobotics.library.Alliance;

public class AutoBuilder {
    
    enum preset{
        NEAR,
        MIDDLE,
        FAR
    }
    private final RobotSystem robot;
    private final FollowerSubsystem follower;

    //Our robot's starting positions
    public Pose CLOSE_START_POSE = new Pose(22.3, 120, Math.toRadians(0));
    public Pose FAR_START_POSE = new Pose(57.38, 8.44, Math.toRadians(0));

    //Poses that we shoot from depending on location
    private Pose CLOSE_SHOOT_POSE = new Pose(27.82,100.783,Math.toRadians(0));
    private Pose FAR_SHOOT_POSE = new Pose(53.716,17.322, Math.toRadians(0));

    //Leave pose for end of auto points
    private Pose CLOSE_LEAVE_POSE = new Pose(27.5,92.5,Math.toRadians(0));
    private Pose FAR_LEAVE_POSE = new Pose(50,23,Math.toRadians(0));
    private Pose RAMP_PARK_POSE = new Pose(20.471,69.988,Math.toRadians(0));

    //For intaking from loading zone
    private Pose LOADING_ZONE_POSE = new Pose(14,11);

    //Positions to start intaking the desired preload stack
    private Pose PRESET_1_START = new Pose(42.342,84.335,Math.toRadians(0));
    private Pose PRESET_2_START = new Pose(42.342,60.2,Math.toRadians(0));
    private Pose PRESET_3_START = new Pose(42.342,35.694, Math.toRadians(0));

    //Positions after intaking the preload stack
    private Pose PRESET_1_END = new Pose(25.721,84.335,Math.toRadians(0));
    private Pose PRESET_2_END = new Pose(25.721,60.2,Math.toRadians(0));
    private Pose PRESET_3_END = new Pose(25.721,35.694,Math.toRadians(0));

    //All paths needed for compiling autonomous sequences
    public PathChain FAR_SHOOT_TO_PRESET_1;
    public PathChain FAR_SHOOT_TO_PRESET_2;
    public PathChain FAR_SHOOT_TO_PRESET_3;

    public PathChain CLOSE_SHOOT_TO_PRESET_1;
    public PathChain CLOSE_SHOOT_TO_PRESET_2;
    public PathChain CLOSE_SHOOT_TO_PRESET_3;

    public PathChain INTAKE_PRESET_1;
    public PathChain INTAKE_PRESET_2;
    public PathChain INTAKE_PRESET_3;

    public PathChain INTAKE_PRESET_1_TO_FAR_SHOOT;
    public PathChain INTAKE_PRESET_2_TO_FAR_SHOOT;
    public PathChain INTAKE_PRESET_3_TO_FAR_SHOOT;

    public PathChain INTAKE_PRESET_1_TO_CLOSE_SHOOT;
    public PathChain INTAKE_PRESET_2_TO_CLOSE_SHOOT;
    public PathChain INTAKE_PRESET_3_TO_CLOSE_SHOOT;

    public PathChain FAR_SHOOT_TO_LEAVE;
    public PathChain CLOSE_SHOOT_TO_LEAVE;

    public PathChain START_TO_STEAL;

    public AutoBuilder(RobotSystem robot, FollowerSubsystem followerSubsystem) {
        this.robot = robot;
        this.follower = followerSubsystem;

        if (Alliance.get() == Alliance.RED) {
            flipPoses();
        }

        buildPaths();
    }

    private void buildPaths() {
        INTAKE_PRESET_1 = pathLinearHeading(PRESET_1_START, PRESET_1_END);
        INTAKE_PRESET_2 = pathLinearHeading(PRESET_2_START, PRESET_2_END);
        INTAKE_PRESET_3 = pathLinearHeading(PRESET_3_START, PRESET_3_END);
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
    private PathChain pathLinearHeading(Pose pose1, Pose pose2){
        return (follower.pathBuilder().addPath(new BezierLine(pose1, pose2))
                .setLinearHeadingInterpolation(pose1.getHeading(),pose2.getHeading())
                .build());
    }

    public PathChain farShootToLoadingZone(double startHeading, double endHeading){
        return (follower.pathBuilder().addPath(new BezierLine(FAR_SHOOT_POSE, LOADING_ZONE_POSE))
                .setLinearHeadingInterpolation(startHeading,endHeading))
                .build();
    }

    public PathChain loadingZoneToFarShoot(double startHeading, double endHeading){
        return (follower.pathBuilder().addPath(new BezierLine(LOADING_ZONE_POSE, FAR_SHOOT_POSE))
                .setLinearHeadingInterpolation(startHeading,endHeading))
                .build();
    }
    
    
    //intake a set of balls
    private Command intakePresets(preset set){
        PathChain intakePath = (set == preset.NEAR ? INTAKE_PRESET_1 : set == preset.MIDDLE ? INTAKE_PRESET_2 : INTAKE_PRESET_3); //select preset to intake
        return new SequentialCommandGroup(
                intake(),
                follower.followPath(intakePath, false),
                spinup(),
                new WaitCommand(250),
                stopIntake()
        );
    }
    private Command nearToPreset(preset set){
        PathChain selectPath = (set == preset.NEAR ? CLOSE_SHOOT_TO_PRESET_1 : set == preset.MIDDLE ? CLOSE_SHOOT_TO_PRESET_2 : CLOSE_SHOOT_TO_PRESET_3);
        return follower.followPath(selectPath, false);
    }
    private Command farToPreset(preset set){
        PathChain selectPath = (set == preset.NEAR ? FAR_SHOOT_TO_PRESET_1 : set == preset.MIDDLE ? FAR_SHOOT_TO_PRESET_2 : FAR_SHOOT_TO_PRESET_3);
        return follower.followPath(selectPath, false);
    }
    private Command presetToNear(preset set){
        PathChain selectPath = (set == preset.NEAR ? INTAKE_PRESET_1_TO_CLOSE_SHOOT : set == preset.MIDDLE ? INTAKE_PRESET_2_TO_CLOSE_SHOOT : INTAKE_PRESET_3_TO_CLOSE_SHOOT);
        return follower.followPath(selectPath, true);
    }
    private Command presetToFar(preset set){
        PathChain selectPath = (set == preset.NEAR ? INTAKE_PRESET_1_TO_FAR_SHOOT : set == preset.MIDDLE ? INTAKE_PRESET_2_TO_FAR_SHOOT: INTAKE_PRESET_3_TO_FAR_SHOOT);
        return follower.followPath(selectPath, true);
    }
    
    public Command cycle(preset set, boolean startNear, boolean endNear){
        Command startPathCMD = (startNear ? nearToPreset(set) : farToPreset(set));
        Command intakePathCMD = intakePresets(set);
        Command endPathCMD = (endNear ? presetToNear(set) : presetToFar(set));
        return new SequentialCommandGroup(
                startPathCMD,
                intakePathCMD,
                endPathCMD
        );
    }
    public Command stealCycle(){
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
    public Command parkNear(){
        return (follower.followPath(CLOSE_SHOOT_TO_LEAVE, true));
    }
    public Command parkFar(){
        return (follower.followPath(FAR_SHOOT_TO_LEAVE, true));
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
        PRESET_1_START = AlliancePoseFlipper.flip(PRESET_1_START);
        PRESET_2_START = AlliancePoseFlipper.flip(PRESET_2_START);
        PRESET_3_START = AlliancePoseFlipper.flip(PRESET_3_START);
        PRESET_1_END = AlliancePoseFlipper.flip(PRESET_1_END);
        PRESET_2_END = AlliancePoseFlipper.flip(PRESET_2_END);
        PRESET_3_END = AlliancePoseFlipper.flip(PRESET_3_END);
        RAMP_PARK_POSE = AlliancePoseFlipper.flip(RAMP_PARK_POSE);
    }
}
