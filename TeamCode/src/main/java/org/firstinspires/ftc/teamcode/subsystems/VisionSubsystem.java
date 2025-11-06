package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
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

@SuppressWarnings("FieldCanBeLocal")
public class VisionSubsystem extends StealthSubsystem {
    private final AprilTagProcessor aprilTagProcessor;

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
                        data.id, data.name, data.tagsize, data.fieldPosition, data.distanceUnit, data.fieldOrientation
                );
            }
            else libraryBuilder.addTag(data);
        }

        AprilTagLibrary tagLibrary = libraryBuilder.build();

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setTagLibrary(tagLibrary)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "aprilTagCamera"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setAutoStopLiveView(true)
                .build();

        if (FtcDashboard.getInstance().isEnabled())
            FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    //Converts from FTCCoordinates to PedroCoordinates
    private Pose poseAdjust(Pose old) {
        return new Pose(72 + old.getY(), 72 - old.getX(), old.getHeading() - (Math.PI / 2));
    }

    // ? Returns the new motif if it has changed since last call
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

    @Override
    public void periodic() {
        ArrayList<AprilTagDetection> latestDetections = aprilTagProcessor.getDetections();
        if (!latestDetections.isEmpty()) {
            for (AprilTagDetection detection : latestDetections) {
                if (detection.id == GOAL_BLUE_ID || detection.id == GOAL_RED_ID) {
                    PoseTracker.updateEstimatedPose(poseAdjust(new Pose(
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
                    )), false);
                }
            }
        }
        telemetry.addData("Detected Motif", Motif.getMotif());
    }
}
