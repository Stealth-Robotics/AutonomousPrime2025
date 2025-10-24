package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.PoseTracker;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.ArrayList;

public class VisionSubsystem extends StealthSubsystem {
    private final AprilTagProcessor aprilTagProcessor;
    private final AprilTagLibrary tagLibrary;
    private final VisionPortal visionPortal;

    //TODO: VERY IMPORTANT TO SET THIS ACCURATELY
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 0, 0, 0, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    private final int MOTIF_GPP_ID = 21, MOTIF_PGP_ID = 22, MOTIF_PPG_ID = 23;
    private final int GOAL_BLUE_ID = 20, GOAL_RED_ID = 24;

    public VisionSubsystem(HardwareMap hardwareMap) {
        AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();
        AprilTagMetadata[] decodeLibrary = AprilTagGameDatabase.getDecodeTagLibrary().getAllTags();

        for (AprilTagMetadata data : decodeLibrary) {
            if (data.fieldOrientation != null && data.fieldPosition != null) {
                libraryBuilder.addTag(
                        data.id, data.name, data.tagsize, poseAdjust(data.fieldPosition), data.distanceUnit, angleAdjust(data.fieldOrientation)
                );
            }
            else libraryBuilder.addTag(data);
        }

        tagLibrary = libraryBuilder.build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setTagLibrary(tagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class,"aprilTagCamera"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640,480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        visionPortal.resumeStreaming();
    }

    private VectorF poseAdjust(VectorF old) {
        return new VectorF(72 + old.getData()[1], 72 - old.getData()[0], old.getData()[2]);
    }

    private Quaternion angleAdjust(Quaternion q) {
        Quaternion rotation = new Quaternion(0.707f,0,0,-0.707f,0);
        q.multiply(rotation,0);
        return(q);
    }

    // ? Returns a new motif if the camera detects a new one
    public Motif.MotifType getUpdatedMotif(Motif.MotifType old) {
        ArrayList<AprilTagDetection> latestDetections = aprilTagProcessor.getFreshDetections();

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

    @Override
    public void periodic() {
        ArrayList<AprilTagDetection> latestDetections = aprilTagProcessor.getDetections();
        for (AprilTagDetection detection : latestDetections) {
            if (detection.id == GOAL_BLUE_ID && Alliance.isBlue() || detection.id == GOAL_RED_ID && Alliance.isRed()) {
                PoseTracker.updateEstimatedPose(new Pose(
                        detection.robotPose.getPosition().x,
                        detection.robotPose.getPosition().y,
                        detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS) // ! Might need to be degrees
                ));
            }
        }
    }
}
