package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@Config
public class DriveSubsystem extends StealthSubsystem {
    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightBack;

    private final PoseEstimator poseEstimator;

    private final GoBildaPinpointDriver pp;

    private double headingOffset = 0.0;

    //Stores the latest call to set the pinpoint's pose, stored in case the pinpoint is busy when the method is called
    private Pose latestPoseSetCall = null;

    public DriveSubsystem(HardwareMap hardwareMap) {
        pp = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        rightBack = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftFront = hardwareMap.get(DcMotorEx.class, "rightBack");

        poseEstimator = PoseEstimator.getInstance();

        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pp.setOffsets(3.375, 8.125, DistanceUnit.INCH);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.FORWARD);
    }

    public void resetPosAndIMU() {
        pp.resetPosAndIMU();
    }

    public void setPose(Pose newPose) {
        if (pp.getDeviceStatus() != GoBildaPinpointDriver.DeviceStatus.READY) {
            latestPoseSetCall = newPose;
        }
        else {
            pp.setPosition(new Pose2D(DistanceUnit.INCH, newPose.getY(), newPose.getX(), AngleUnit.RADIANS, newPose.getHeading()));
            latestPoseSetCall = null;
        }
    }

    public void resetToPosition(int x, int y, int theta) {
        pp.setPosition(new Pose2D(DistanceUnit.INCH, y, x, AngleUnit.DEGREES, theta));
    }

    public double getHeading() {
        return pp.getHeading(AngleUnit.RADIANS);
    }

    public void resetHeading() {
        headingOffset = -pp.getHeading(AngleUnit.RADIANS);
    }

    public void drive(double x, double y, double rot) {
        double heading = getHeading() + headingOffset;
        double dx = x * Math.cos(-heading) - y * Math.sin(-heading);
        double dy = x * Math.sin(-heading) + y * Math.cos(-heading);

        dx *= 1.1; // Counteract imperfect strafing

        double denominator = Math.max(Math.abs(dy) + Math.abs(dx) + Math.abs(rot), 1);
        double leftFrontPower = (dy + dx + rot) / denominator;
        double leftBackPower = (dy - dx + rot) / denominator;
        double rightFrontPower = (dy - dx - rot) / denominator;
        double rightBackPower = (dy + dx - rot) / denominator;

        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightFront.setPower(rightFrontPower);
        rightBack.setPower(rightBackPower);
    }

    public Command driveTeleop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return this.run(() -> drive(x.getAsDouble(), y.getAsDouble(), rot.getAsDouble()));
    }

    @Override
    public void periodic() {
        pp.update();

        // Update pose if there is a pose queued from a previous setPose method call
        if (pp.getDeviceStatus() == GoBildaPinpointDriver.DeviceStatus.READY && latestPoseSetCall != null)
            setPose(latestPoseSetCall);

        boolean updatePinpointPose =
                poseEstimator.update(new Pose(
                pp.getPosY(DistanceUnit.INCH),
                pp.getPosX(DistanceUnit.INCH),
                getHeading()
        ));

        if (updatePinpointPose)
            setPose(poseEstimator.getRobotPose());

        telemetry.addLine("----drive----");
        telemetry.addData("y", pp.getPosY(DistanceUnit.INCH));
        telemetry.addData("x", pp.getPosX(DistanceUnit.INCH));
        telemetry.addData("θ", AngleUnit.RADIANS.toDegrees(getHeading()));
    }
}
