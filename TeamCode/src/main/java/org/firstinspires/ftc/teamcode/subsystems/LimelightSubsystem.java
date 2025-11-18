package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.Motif;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class LimelightSubsystem extends StealthSubsystem {
    private final Limelight3A limelight;

    private final PoseEstimator poseEstimator;

    private final int MOTIF_GPP_ID = 21, MOTIF_PGP_ID = 22, MOTIF_PPG_ID = 23;
    private final int GOAL_BLUE_ID = 20, GOAL_RED_ID = 24;

    private LLResult latestResult = null;

    public LimelightSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        limelight.start();
        limelight.pipelineSwitch(0);

        poseEstimator = PoseEstimator.getInstance();
    }

    @Override
    public void periodic() {
        latestResult = limelight.getLatestResult();
        if (latestResult == null || !latestResult.isValid()) {
            latestResult = null;
        }

        if (latestResult != null && latestResult.isValid()) {
            for (FiducialResult tag : latestResult.getFiducialResults()) {
                int tagID = tag.getFiducialId();
                if (tagID == GOAL_BLUE_ID || tagID == GOAL_RED_ID) {
                    //Update poseEstimator with limelight robot pose estimation in field space
                    //Ignore heading because pinpoint is accurate enough for that
                    //TODO: Figure in what coordinates the limelight gives us and convert to Pedro Coordinates if needed
                    poseEstimator.setLimelightPoseUpdate(
                            new Pose(
                                    tag.getRobotPoseFieldSpace().getPosition().x,
                                    tag.getRobotPoseFieldSpace().getPosition().y
                            )
                    );
                }

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
        telemetry.addData("distanceToGoal", poseEstimator.getDistanceFromGoal());
        telemetry.addData("motif", Motif.getMotif());
    }
}
