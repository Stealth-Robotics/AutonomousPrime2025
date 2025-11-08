package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.PoseTracker;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.stealthrobotics.library.StealthSubsystem;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import java.util.ArrayList;
import java.util.Arrays;

@SuppressWarnings("FieldCanBeLocal")
@Config
public class VisionSubsystem extends StealthSubsystem {
    private final AprilTagProcessor aprilTagProcessor;
    private final VisionPortal visionPortal;

    private final int MOTIF_GPP_ID = 21, MOTIF_PGP_ID = 22, MOTIF_PPG_ID = 23;
    private final int GOAL_BLUE_ID = 20, GOAL_RED_ID = 24;

    public VisionSubsystem(HardwareMap hardwareMap) {
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
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .build();
    }

    @Override
    public void periodic() {
        telemetry.addData("fps", visionPortal.getFps());
        ArrayList<AprilTagDetection> latestDetections = aprilTagProcessor.getDetections();
        ArrayList<Integer> tagIds = new ArrayList<>();
        for (AprilTagDetection detection : latestDetections) {
            tagIds.add(detection.id);
        }
        telemetry.addData("tagIds", tagIds);
        if (tagIds.contains(24)) {
            AprilTagDetection tag = latestDetections.get(0);
            telemetry.addData("range", tag.ftcPose.range);
            telemetry.addData("headingOffset", tag.ftcPose.bearing);
        }
    }
}
