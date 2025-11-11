package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
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

    private final int ALLIANCE_GOAL_ID;

    private LLResult latestResult = null;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        limelight.start();
        limelight.pipelineSwitch(0);

        ALLIANCE_GOAL_ID = (Alliance.get() == Alliance.BLUE) ? GOAL_BLUE_ID : GOAL_RED_ID;
    }

    private double getDistanceToGoal() {
        double heightBetweenAprilTagAndLimelight = GOAL_TAG_HEIGHT_INCHES - LIMELIGHT_DISTANCE_OFF_FLOOR_INCHES;
        return heightBetweenAprilTagAndLimelight / Math.tan(Math.toRadians(LIMELIGHT_MOUNT_ANGLE_DEGREES + latestResult.getTy()));
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 5; i++) {
            latestResult = limelight.getLatestResult();
            if (latestResult != null && latestResult.isValid()) {
                break;
            }
            else latestResult = null;
        }

        if (latestResult != null && latestResult.isValid()) {
            for (FiducialResult tag : latestResult.getFiducialResults()) {
                int tagID = tag.getFiducialId();
                if (tagID == ALLIANCE_GOAL_ID) {
                    LatestGoalData.updateGoalData(latestResult.getTx(), getDistanceToGoal());
                }
                else LatestGoalData.tagInvisible();

                //Check for motif
                if (Motif.getMotif() == Motif.MotifType.NULL) {
                    if (tagID == MOTIF_GPP_ID)
                        Motif.setMotif(Motif.MotifType.GPP);
                    else if (tagID == MOTIF_PGP_ID)
                        Motif.setMotif(Motif.MotifType.PGP);
                    else if (tagID == MOTIF_PPG_ID)
                        Motif.setMotif(Motif.MotifType.PPG);
                }
            }
        }

        telemetry.addLine("----vision----");
        telemetry.addData("seesGoal", LatestGoalData.canSeeTag());
        telemetry.addData("distanceToGoal", LatestGoalData.getDistanceFromGoal());
        telemetry.addData("motif", Motif.getMotif());
    }
}
