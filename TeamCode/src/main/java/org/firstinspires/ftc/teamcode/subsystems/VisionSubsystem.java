package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes.*;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.enums.MotifType;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.ArrayList;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class VisionSubsystem extends StealthSubsystem {
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTagProcessor;

    private final Alliance alliance;

    private final ElapsedTime restTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);

    private final double REST_SECONDS = 1.5;

    //Position and rotation of the camera relative to the robot's origin (in inches)
    private final Position cameraPosition = new Position(DistanceUnit.INCH, 3.75, 8.25, 9.2, 0);
    private final YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES, 0,-90, 180, 0);

    private final PoseEstimator poseEstimator;

    private final int MOTIF_GPP_ID = 21, MOTIF_PGP_ID = 22, MOTIF_PPG_ID = 23;
    private final int GOAL_BLUE_ID = 20, GOAL_RED_ID = 24;

    public VisionSubsystem(HardwareMap hardwareMap) {
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .setTagLibrary(AprilTagGameDatabase.getDecodeTagLibrary())
                .setLensIntrinsics(549.651, 549.651, 317.108, 236.644) //From FTC discord (veer)
                .build();

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "arducam"))
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setAutoStopLiveView(true)
                .build();

        poseEstimator = PoseEstimator.getInstance();

        alliance = Alliance.get();

        restTimer.reset();
    }

    // Takes a pose in FTC Coordinates (origin (0, 0)) to Pedro Coordinates (origin (72, 72))
    private Pose ftcToPedroCoordinates(Pose ftcPose) {
        return new Pose(72 + ftcPose.getY(), 72 - ftcPose.getX(), ftcPose.getHeading());
    }

    @Override
    public void periodic() {
        ArrayList<AprilTagDetection> detections = aprilTagProcessor.getDetections();

        if (!detections.isEmpty()) {
            for (AprilTagDetection detection : detections) {
                if (restTimer.seconds() >= REST_SECONDS) {
                    if (alliance == Alliance.BLUE && detection.id == GOAL_BLUE_ID || alliance == Alliance.RED && detection.id == GOAL_RED_ID) {
                        poseEstimator.updateWithNewPose(ftcToPedroCoordinates(new Pose(
                                detection.robotPose.getPosition().x,
                                detection.robotPose.getPosition().y,
                                detection.robotPose.getOrientation().getYaw(AngleUnit.RADIANS)
                        )));
                    }
                    restTimer.reset();
                }

                if (detection.id == MOTIF_GPP_ID) {
                    Motif.setMotif(MotifType.GPP);
                }
                else if (detection.id == MOTIF_PGP_ID) {
                    Motif.setMotif(MotifType.PGP);
                }
                else if (detection.id == MOTIF_PPG_ID) {
                    Motif.setMotif(MotifType.PPG);
                }
            }
        }

        telemetry.addLine("----vision----");
        telemetry.addData("cameraState", visionPortal.getCameraState());
        telemetry.addData("fps", visionPortal.getFps());
        telemetry.addData("seesGoal", restTimer.seconds() < REST_SECONDS);
        telemetry.addData("distanceToGoal", poseEstimator.getDistanceFromGoal());
        telemetry.addData("motif", Motif.getMotif());

    }
}
