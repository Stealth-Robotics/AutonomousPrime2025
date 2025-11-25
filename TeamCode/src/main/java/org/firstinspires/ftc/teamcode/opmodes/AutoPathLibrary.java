package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.pedroPathing.AlliancePoseFlipper;
import org.stealthrobotics.library.Alliance;

public class AutoPathLibrary {
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

    //All paths for compiling partner compatible autonomous sequences
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

    public AutoPathLibrary() {
        if (Alliance.get() == Alliance.RED) {
            flipPoses();
        }

        buildPaths();
    }

    private void buildPaths() {

    }

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
