package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.PoseTracker;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.function.DoubleSupplier;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@Config
public class DriveSubsystem extends StealthSubsystem {
    private final DcMotorEx leftFront;
    private final DcMotorEx leftBack;
    private final DcMotorEx rightFront;
    private final DcMotorEx rightBack;

    private final GoBildaPinpointDriver pp;

    public DriveSubsystem(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        rightBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        pp = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pp.setOffsets(3, -5, DistanceUnit.INCH);
        pp.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pp.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pp.resetPosAndIMU();
    }

    public double getHeading() {
        return pp.getHeading(AngleUnit.RADIANS);
    }

    public void resetHeading() {
        pp.resetPosAndIMU();
    }

    public void drive(double x, double y, double rot) {
        double heading = getHeading();
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

        telemetry.addData("leftFrontPower", leftFrontPower);
        telemetry.addData("leftBackPower", leftBackPower);
        telemetry.addData("rightFrontPower", rightFrontPower);
        telemetry.addData("rightBackPower", rightBackPower);

    }

    public Command driveTeleop(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rot) {
        return this.run(() -> drive(x.getAsDouble(), y.getAsDouble(), rot.getAsDouble()));
    }

    @Override
    public void periodic() {
        pp.update();

        Pose2D pose = pp.getPosition();
        PoseTracker.updateEstimatedPose(
                new Pose(-pose.getX(DistanceUnit.INCH), -pose.getY(DistanceUnit.INCH), getHeading()), true
        );

        telemetry.addLine("-----drive-----");
        telemetry.addData("x", PoseTracker.getEstimatedPose().getX());
        telemetry.addData("y", PoseTracker.getEstimatedPose().getY());
        telemetry.addData("θ", AngleUnit.RADIANS.toDegrees(getHeading()));
    }
}
