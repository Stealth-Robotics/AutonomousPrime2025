package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.LatestGoalData;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class LimelightSubsystem extends StealthSubsystem {
    private final Limelight3A limelight;

    private final double LIMELIGHT_MOUNT_ANGLE_DEGREES = 20.0;
    private final double LIMELIGHT_DISTANCE_OFF_FLOOR_INCHES = 15.25;
    private final double GOAL_TAG_HEIGHT_INCHES = 29.5;

    private final int MOTIF_GPP_ID = 21, MOTIF_PGP_ID = 22, MOTIF_PPG_ID = 23;
    private final int GOAL_BLUE_ID = 20, GOAL_RED_ID = 24;

    private LLResult latestResult;
    private Pipeline currentPipeline = Pipeline.OBELISK;

    private enum Pipeline {
        GOAL_BLUE(0),
        GOAL_RED(1),
        OBELISK(3);

        final int pipelineID;
        Pipeline(int id) {
            pipelineID = id;
        }
    }

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        limelight.start();
        setMotifTracking();
    }

    //Call once we want to start goal tracking
    public void setGoalTracking() {
        Pipeline goalPipeline = (Alliance.get() == Alliance.BLUE) ? Pipeline.GOAL_BLUE : Pipeline.GOAL_RED;
        currentPipeline = goalPipeline;
        limelight.pipelineSwitch(goalPipeline.pipelineID);
    }

    public void setMotifTracking() {
        Pipeline goalPipeline = Pipeline.OBELISK;
        currentPipeline = goalPipeline;
        limelight.pipelineSwitch(goalPipeline.pipelineID);
    }

    public Motif.MotifType getUpdatedMotif(Motif.MotifType old) {
        if (latestResult != null && latestResult.isValid()) {
            for (LLResultTypes.FiducialResult tag : latestResult.getFiducialResults()) {
                if (tag.getFiducialId() == MOTIF_GPP_ID && old != Motif.MotifType.GPP) {
                    return Motif.MotifType.GPP;
                }
                else if (tag.getFiducialId() == MOTIF_PGP_ID && old != Motif.MotifType.PGP) {
                    return Motif.MotifType.PGP;
                }
                else if (tag.getFiducialId() == MOTIF_PPG_ID && old != Motif.MotifType.PPG) {
                    return Motif.MotifType.PPG;
                }
            }
        }
        return old;
    }

    public void updateGoalEstimates() {
        if (latestResult != null && latestResult.isValid()) {
            LatestGoalData.updateGoalData(latestResult.getTx(), getDistanceToGoal());
        }
        else LatestGoalData.tagInvisible();
    }

    private double getDistanceToGoal() {
        double heightBetweenAprilTagAndLimelight = GOAL_TAG_HEIGHT_INCHES - LIMELIGHT_DISTANCE_OFF_FLOOR_INCHES;
        return heightBetweenAprilTagAndLimelight / Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEGREES + latestResult.getTy()));
    }

    @Override
    public void periodic() {
        //Update more than once per loop
        for (int i = 0; i < 5; i++) {
            latestResult = limelight.getLatestResult();
            if (latestResult != null && latestResult.isValid()) {
                break;
            }
        }

        //Only update goal estimates if
        if (currentPipeline.pipelineID != Pipeline.OBELISK.pipelineID)
            updateGoalEstimates();

        telemetry.addLine("----vision----");
        telemetry.addData("pipeline", currentPipeline);
        telemetry.addData("seesGoal", (latestResult != null && latestResult.isValid()));
        telemetry.addData("distanceToGoal", (latestResult != null) ? getDistanceToGoal() : "unknown");
        telemetry.addData("motif", Motif.getMotif());
    }
}
