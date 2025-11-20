package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.ArrayList;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class VisionSubsystem extends StealthSubsystem {
//    private final Limelight3A limelight;

    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    private final Position cameraPosition = new Position(DistanceUnit.INCH, -4.97790, -7.87748, -9.53956, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private final PoseEstimator poseEstimator;

    private final int MOTIF_GPP_ID = 21, MOTIF_PGP_ID = 22, MOTIF_PPG_ID = 23;
    private final int GOAL_BLUE_ID = 20, GOAL_RED_ID = 24;

//    private LLResult latestResult = null;

    public VisionSubsystem(HardwareMap hardwareMap) {
//        limelight = hardwareMap.get(Limelight3A.class,"limelight");

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "arducam"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .build();

        if (FtcDashboard.getInstance().isEnabled())
            FtcDashboard.getInstance().startCameraStream(visionPortal, 0);

//        limelight.start();
//        limelight.pipelineSwitch(0);

        poseEstimator = PoseEstimator.getInstance();
    }

    private Pose ftcToPedroCoordinates(Pose oldPose) {
        return new Pose(72 + oldPose.getY(), 72 - oldPose.getX(), oldPose.getHeading() - (Math.PI / 2));
    }

    @Override
    public void periodic() {
        ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (!detections.isEmpty()) {
            for (AprilTagDetection detection : detections) {
                if (detection.id == GOAL_BLUE_ID || detection.id == GOAL_RED_ID) {
                    poseEstimator.updateWithNewPose(ftcToPedroCoordinates(new Pose(
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
                    )));
                }

                if (detection.id == MOTIF_GPP_ID)
                    Motif.setMotif(Motif.MotifType.GPP);
                else if (detection.id == MOTIF_PGP_ID)
                    Motif.setMotif(Motif.MotifType.PGP);
                else if (detection.id == MOTIF_PPG_ID)
                    Motif.setMotif(Motif.MotifType.PPG);
            }
        }

        telemetry.addLine("----vision----");
        telemetry.addData("distanceToGoal", poseEstimator.getDistanceFromGoal());
        telemetry.addData("motif", Motif.getMotif());
    }
}
