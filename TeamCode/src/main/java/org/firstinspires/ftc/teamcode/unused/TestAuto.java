//package org.firstinspires.ftc.teamcode.opmodes;
//
//import com.arcrobotics.ftclib.command.Command;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
//import org.firstinspires.ftc.teamcode.PoseSupplier;
//import org.firstinspires.ftc.teamcode.pedroPathing.AlliancePoseFlipper;
//import org.firstinspires.ftc.teamcode.subsystems.FollowerSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.LimelightSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SpindexerSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.TurretSubsystem;
//import org.stealthrobotics.library.Alliance;
//import org.stealthrobotics.library.opmodes.StealthOpMode;
//
//public class TestAuto extends StealthOpMode {
//    private FollowerSubsystem follower;
//
//    private ShooterSubsystem shooter;
//    private IntakeSubsystem intake;
//    private TurretSubsystem turret;
//    private SpindexerSubsystem spindexer;
//    private LimelightSubsystem limelight;
//
//    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));
//
//    @Override
//    public void initialize() {
//        follower = new FollowerSubsystem(hardwareMap);
//
//        shooter = new ShooterSubsystem(hardwareMap);
//        intake = new IntakeSubsystem(hardwareMap);
//        turret = new TurretSubsystem(hardwareMap, new PoseSupplier(() -> follower.getPose().getX(), () -> follower.getPose().getY(), () -> AngleUnit.RADIANS.toDegrees(follower.getPose().getHeading())));
//        spindexer = new SpindexerSubsystem(hardwareMap, intake);
//        limelight = new LimelightSubsystem(hardwareMap);
//
//        //Add all poses in the autonomous here
//        AlliancePoseFlipper.flip(Alliance.get(), new Pose[]{
//                startPose
//        });
//
//        follower.setStartingPose(startPose);
//    }
//
//    @Override
//    public Command getAutoCommand() {
//        return new SequentialCommandGroup();
//    }
//
//    @SuppressWarnings("unused")
//    @Autonomous(name = "RedTestAuto", group = "Red")
//    public static class RedTestAuto extends TestAuto {
//    }
//
//    @SuppressWarnings("unused")
//    @Autonomous(name = "BlueTestAuto", group = "Blue")
//    public static class BlueTestAuto extends TestAuto {
//    }
//}
