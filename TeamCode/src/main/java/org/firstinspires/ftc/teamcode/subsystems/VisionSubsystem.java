package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.LatestGoalData;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import java.util.ArrayList;

@SuppressWarnings("FieldCanBeLocal")
public class VisionSubsystem extends StealthSubsystem {
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    private final Alliance alliance;

    private final int MOTIF_GPP_ID = 21, MOTIF_PGP_ID = 22, MOTIF_PPG_ID = 23;
    private final int GOAL_BLUE_ID = 20, GOAL_RED_ID = 24;

    private ArrayList<AprilTagDetection> detections = new ArrayList<>();

    public VisionSubsystem(HardwareMap hardwareMap) {
        alliance = Alliance.get();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644) //From FTC discord (veer)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "aprilTagCamera"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setAutoStopLiveView(true)
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    //Returns the new motif if it has changed since last call
    public Motif.MotifType getUpdatedMotif(Motif.MotifType old) {
        ArrayList<AprilTagDetection> latestDetections = aprilTagProcessor.getDetections();

        for (AprilTagDetection detection : latestDetections) {
            if (detection.id == MOTIF_PPG_ID && old != Motif.MotifType.PPG)
                return Motif.MotifType.PPG;
            else if (detection.id == MOTIF_GPP_ID && old != Motif.MotifType.GPP)
                return Motif.MotifType.GPP;
            else if (detection.id == MOTIF_PGP_ID && old != Motif.MotifType.PGP)
                return Motif.MotifType.PGP;
        }

        return old; //No new motif found
    }

    private void updateGoalEstimates() {
        if (!detections.isEmpty()) {
            for (AprilTagDetection detection : detections) {
                if (alliance == Alliance.BLUE && detection.id == GOAL_BLUE_ID || alliance == Alliance.RED &&  detection.id == GOAL_RED_ID) {
                    LatestGoalData.updateGoalData(detection.ftcPose.bearing, detection.ftcPose.range);
                }
                else LatestGoalData.tagInvisible();
            }
        }
    }

    @Override
    public void periodic() {
        //Update april tag detections each loop
        detections = aprilTagProcessor.getDetections();

        updateGoalEstimates();

        telemetry.addLine("----vision----");
        telemetry.addData("fps", visionPortal.getFps());
        telemetry.addData("motif", Motif.getMotif());
    }
}
