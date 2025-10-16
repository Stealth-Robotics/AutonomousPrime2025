package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.stealthrobotics.library.StealthSubsystem;

public class VisionSubsystem extends StealthSubsystem {
    private final AprilTagProcessor aprilTagProcessor;
    private final AprilTagLibrary tagLibrary;
    private final VisionPortal visionPortal;

    private final int MOTIF_GPP_ID = 21, MOTIF_PGP_ID = 22, MOTIF_PPG_ID = 23,
            GOAL_BLUE_ID = 20, GOAL_RED_ID = 24;

    private VectorF poseAdjust(VectorF old) {
        return new VectorF(72 + old.getData()[1], 72 - old.getData()[0], old.getData()[2]);
    }

    private Quaternion angleAdjust(Quaternion q) {
        Quaternion rotation = new Quaternion(0.707f,0,0.707f,0,0);
        q.multiply(rotation,0);
        return(q);
    }

    public VisionSubsystem(HardwareMap hardwareMap) {
        //TODO: Add apriltag data
        AprilTagLibrary.Builder libraryBuilder = new AprilTagLibrary.Builder();
        AprilTagMetadata[] decodeLibrary = AprilTagGameDatabase.getDecodeTagLibrary().getAllTags();
        for(AprilTagMetadata data : decodeLibrary){
            if(data.fieldOrientation != null && data.fieldPosition != null) {
                libraryBuilder.addTag(data.id, data.name, data.tagsize, poseAdjust(data.fieldPosition), data.distanceUnit, angleAdjust(data.fieldOrientation));
            } else {
                libraryBuilder.addTag(data);
            }
        }
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
