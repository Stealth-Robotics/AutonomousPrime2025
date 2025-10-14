package org.firstinspires.ftc.teamcode.subsystems;

import android.graphics.Canvas;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.ArrayList;

public class VisionSubsystem extends StealthSubsystem {
    private final AprilTagProcessor aprilTagProcessor;
    private final AprilTagLibrary tagLibrary;
    private final VisionPortal visionPortal;

    private final int MOTIF_GPP_ID = 21, MOTIF_PGP_ID = 22, MOTIF_PPG_ID = 23,
            GOAL_BLUE_ID = 20, GOAL_RED_ID = 24;

    public VisionSubsystem(HardwareMap hardwareMap){
        //TODO: Add apriltag data
        tagLibrary = new AprilTagLibrary.Builder()
                .addTags(AprilTagGameDatabase.getDecodeTagLibrary())
                .addTag(1,"Awesome",6.5, DistanceUnit.INCH)
                .build();
        aprilTagProcessor = new AprilTagProcessor.Builder()
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
        visionPortal.setProcessorEnabled(aprilTagProcessor,true);

    }
}
