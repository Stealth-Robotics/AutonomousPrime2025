package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.enums.MotifType;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.math.filter.Debouncer;

import java.util.List;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class VisionSubsystem extends StealthSubsystem {
    private final Limelight3A limelight;

    private final int MOTIF_GPP_ID = 21, MOTIF_PGP_ID = 22, MOTIF_PPG_ID = 23;
    private final int GOAL_BLUE_ID = 20, GOAL_RED_ID = 24;

    private final int GOAL_ID;

    private boolean seesGoal = false;
    private double tagOffset = 0.0;

    public VisionSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(250);
        limelight.pipelineSwitch(0);
        limelight.start();

        GOAL_ID = (Alliance.get() == Alliance.BLUE) ? GOAL_BLUE_ID : GOAL_RED_ID;
    }

    public boolean seesGoal() {
        return seesGoal;
    }

    public double getTagOffset() {
        return tagOffset;
    }

    @Override
    public void periodic() {
        LLResult latestTagResults = limelight.getLatestResult();

        boolean seenGoal = false;
        if (latestTagResults != null && latestTagResults.isValid()) {
            List<FiducialResult> tags = latestTagResults.getFiducialResults();
            for (FiducialResult tag : tags) {
                int id = tag.getFiducialId();

                if (id == GOAL_ID) {
                    tagOffset = tag.getTargetXDegrees();
                    seenGoal = true;
                }

                switch (id) {
                    case MOTIF_GPP_ID:
                        Motif.setMotif(MotifType.GPP);
                        break;
                    case MOTIF_PGP_ID:
                        Motif.setMotif(MotifType.PGP);
                        break;
                    case MOTIF_PPG_ID:
                        Motif.setMotif(MotifType.PPG);
                        break;
                }
            }
        }

        seesGoal = (seenGoal) ? true : false;

        telemetry.addData("Limelight FPS", limelight.getStatus().getFps());
        telemetry.addData("Can See Goal", seesGoal);
        telemetry.addData("Distance To Goal", PoseEstimator.getInstance().getDistanceFromGoal());
    }
}
