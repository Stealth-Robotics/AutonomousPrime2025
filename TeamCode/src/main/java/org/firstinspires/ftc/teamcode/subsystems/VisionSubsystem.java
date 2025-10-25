package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvWebcam;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import java.util.ArrayList;
import java.util.Arrays;

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
                        data.id, data.name, data.tagsize, data.fieldPosition, data.distanceUnit, data.fieldOrientation
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

        FtcDashboard.getInstance().startCameraStream(visionPortal, 0);
    }

    private Pose poseAdjust(Pose old) {
        return new Pose(72 + old.getY(), 72 - old.getX(), old.getHeading() - (Math.PI / 2));
    }

    public static double[] quaternionToEuler(Quaternion q) {
        double w = q.w;
        double x = q.x;
        double y = q.y;
        double z = q.z;
        // Normalize the quaternion
        double norm = Math.sqrt(w*w + x*x + y*y + z*z);
        w /= norm;
        x /= norm;
        y /= norm;
        z /= norm;

        // Calculate Euler angles
        double roll = Math.atan2(2.0 * (w*x + y*z), 1.0 - 2.0 * (x*x + y*y));

        double sinPitch = 2.0 * (w*y - z*x);
        // Clamp to handle numerical inaccuracies
        sinPitch = Math.max(-1.0, Math.min(1.0, sinPitch));
        double pitch = Math.asin(sinPitch);

        double yaw = Math.atan2(2.0 * (w*z + x*y), 1.0 - 2.0 * (y*y + z*z));

        return new double[] { roll * 180.0 / Math.PI, pitch * 180.0 / Math.PI, yaw * 180.0 / Math.PI}; // in radians
    }

    private Quaternion eulerToQuaternion(float rollDeg, float pitchDeg, float yawDeg) {
        float roll  = (float) Math.toRadians(rollDeg);
        float pitch = (float) Math.toRadians(pitchDeg);
        float yaw   = (float) Math.toRadians(yawDeg);

        float cy = (float) Math.cos(yaw * 0.5);
        float sy = (float) Math.sin(yaw * 0.5);
        float cp = (float) Math.cos(pitch * 0.5);
        float sp = (float) Math.sin(pitch * 0.5);
        float cr = (float) Math.cos(roll * 0.5);
        float sr = (float) Math.sin(roll * 0.5);

        float w = cr * cp * cy + sr * sp * sy;
        float x = sr * cp * cy - cr * sp * sy;
        float y = cr * sp * cy + sr * cp * sy;
        float z = cr * cp * sy - sr * sp * cy;

        return new Quaternion(w, x, y, z, System.nanoTime());
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
        TelemetryPacket packet = new TelemetryPacket();

        ArrayList<AprilTagDetection> latestDetections = aprilTagProcessor.getDetections();
        if (!latestDetections.isEmpty()) {
            for (AprilTagDetection detection : latestDetections) {
                if (detection.id == GOAL_BLUE_ID || detection.id == GOAL_RED_ID) {
                    PoseTracker.updateEstimatedPose(poseAdjust(new Pose(
                            detection.robotPose.getPosition().x,
                            detection.robotPose.getPosition().y,
                            detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
                    )));
//                    packet.fieldOverlay()
//                            .drawImage("/dash/decode_field.png", 0, 0, 144, 144);
//
//                    packet.fieldOverlay()
//                            .setFill("black")
//                            .fillRect(-detection.robotPose.getPosition().x - 8, -detection.robotPose.getPosition().y - 8, 18, 16);
                }
            }
        }

//        packet.put("x (pedro)", estimatedPose.getX());
//        packet.put("y (pedro) ", estimatedPose.getY());
        packet.put("heading (degrees) (pedro)", (PoseTracker.getEstimatedPose().getHeading() * 180.0) / Math.PI);
//
        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboard.sendTelemetryPacket(packet);
    }
}
