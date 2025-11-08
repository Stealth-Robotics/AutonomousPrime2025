//package org.firstinspires.ftc.teamcode.subsystems;
//
//import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;
//

// ! Disabled because we aren't using Limelight currently

//import com.qualcomm.hardware.limelightvision.LLResult;
//import com.qualcomm.hardware.limelightvision.LLResultTypes;
//import com.qualcomm.hardware.limelightvision.Limelight3A;
//import com.qualcomm.robotcore.hardware.HardwareMap;
//
//import org.firstinspires.ftc.teamcode.Motif;
//import org.stealthrobotics.library.Alliance;
//import org.stealthrobotics.library.StealthSubsystem;
//
//public class LimelightSubsystem extends StealthSubsystem {
//    Limelight3A limelight;
//    private final int MOTIF_GPP_ID = 21, MOTIF_PGP_ID = 22, MOTIF_PPG_ID = 23;
//    private final int GOAL_BLUE_ID = 20, GOAL_RED_ID = 24;
//    private int ALLIANCE_GOAL_TAG_ID;
//    public LimelightSubsystem(HardwareMap hardwareMap) {
//        limelight = hardwareMap.get(Limelight3A.class,"limelight");
//        limelight.pipelineSwitch(0);
//        limelight.start();
//        ALLIANCE_GOAL_TAG_ID = (Alliance.get() == Alliance.BLUE) ? GOAL_BLUE_ID : GOAL_RED_ID;
//    }
//    public Motif.MotifType getUpdatedMotif(Motif.MotifType old) {
//        LLResult result = limelight.getLatestResult();
//        if(result.isValid()){
//            for(LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
//                if(tag.getFiducialId() == MOTIF_GPP_ID && old != Motif.MotifType.GPP) {
//                    return Motif.MotifType.GPP;
//                } else if (tag.getFiducialId() == MOTIF_PGP_ID && old != Motif.MotifType.PGP) {
//                    return Motif.MotifType.PGP;
//                } else if (tag.getFiducialId() == MOTIF_PPG_ID && old != Motif.MotifType.PPG) {
//                    return Motif.MotifType.PPG;
//                }
//            }
//        }
//        return old;
//    }
//    public Double getAngleToGoal() {
//        LLResult result = limelight.getLatestResult();
//        if(result.isValid()) {
//            for(LLResultTypes.FiducialResult tag : result.getFiducialResults()) {
//                if(tag.getFiducialId() == ALLIANCE_GOAL_TAG_ID) {
//                    return tag.getTargetXDegrees();
//                }
//            }
//        }
//        return null;
//    }
//    @Override
//    public void periodic(){
//        telemetry.addData("Motif",Motif.getMotif());
//    }
//}
