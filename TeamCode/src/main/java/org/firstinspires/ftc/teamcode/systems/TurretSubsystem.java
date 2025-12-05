package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.enums.TurretState;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.SquIDController;
import org.stealthrobotics.library.StealthSubsystem;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class TurretSubsystem extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final PIDController odoPID;
    private final PIDController apriltagPID;

    private final PoseEstimator poseEstimator;

    private final double farTurretOffset;

    private double encoderOffset = 0.0;

    private TurretState state = TurretState.IDLE;

    private double offsetFromTag = 0.0;

    public static double odo_kP = 0.03;
    public static double odo_kI = 0.06;
    public static double odo_kD = 0.0;
    public static double odo_kS = 0.08;

    public static double apriltag_kP = 0.02;
    public static double apriltag_kI = 0.0;
    public static double apriltag_kD = 0.0;
    public static double apriltag_kS = 0.08;

    private final double TICKS_PER_REVOLUTION = 4 * 537.7; // (output ratio) * PPR = 4 * 537.7

    private final double MAX_DEGREES_RIGHT = 170;
    private final double MAX_DEGREES_LEFT = -170;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        poseEstimator = PoseEstimator.getInstance();

        odoPID = new PIDController(odo_kP, odo_kI, odo_kD);
        apriltagPID = new PIDController(apriltag_kP, apriltag_kI, apriltag_kD);

        if (Alliance.isRed())
            farTurretOffset = -1;
        else
            farTurretOffset = 8;

        switchToOdometryControl();
        resetEncoder();
    }

    public Command setState(TurretState newState) {
        return this.runOnce(() -> state = newState);
    }

    public TurretState getState() {
        return state;
    }

    private void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setPower(double power) {
        turretMotor.setPower(power);
    }

    //Essentially re-zeros the motor encoder without homing
    public void setEncoderOffset(double encoderOffset) {
        this.encoderOffset = encoderOffset;
    }

    //Returns the raw, unaltered ticks of the motor
    public int getRawTicks() {
        return turretMotor.getCurrentPosition();
    }

    //Returns the ticks based off of where 0 should be
    private double getCurrentTicks() {
        return turretMotor.getCurrentPosition() + encoderOffset;
    }

    private double getCurrentDegrees() {
        return (getCurrentTicks() / TICKS_PER_REVOLUTION) * 360;
    }

    public void switchToHome() {
        state = TurretState.HOME;
        odoPID.reset();
    }

    public void switchToObelisk() {
        state = TurretState.OBELISK;
        odoPID.reset();
    }

    public void switchToOdometryControl() {
        state = TurretState.ODOMETRY;
        odoPID.reset();
    }

    public void switchToApriltagControl() {
        state = TurretState.APRILTAG;
        apriltagPID.reset();
    }

    public void updateOffsetFromTag(double newOffset) {
        offsetFromTag = newOffset;
    }

    @Override
    public void periodic() {
        //Constant offset for far shooting (y less than 48 inches)
        double constantOffset = 0;
        if (poseEstimator.getRobotPose().getY() < 48) {
            constantOffset = farTurretOffset;
        }

        switch (state) {
            case HOME:
                double odoOutput = odoPID.calculate(getCurrentDegrees(), 0);
                setPower(odoOutput + (odo_kS * Math.signum(odoPID.getPositionError())));
                break;

            case OBELISK:
                odoOutput = odoPID.calculate(getCurrentDegrees(), MathFunctions.clamp(poseEstimator.getObeliskTurretTargetAngle(), MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
                setPower(odoOutput + (odo_kS * Math.signum(odoPID.getPositionError())));
                break;

            case ODOMETRY:
                odoOutput = odoPID.calculate(getCurrentDegrees(), MathFunctions.clamp(poseEstimator.getTurretTargetAngle() - constantOffset, MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
                setPower(odoOutput + (odo_kS * Math.signum(odoPID.getPositionError())));
                break;

            case APRILTAG:
                double offset = -offsetFromTag + constantOffset;

                if (getCurrentDegrees() - offset < MAX_DEGREES_LEFT)
                    offset = MAX_DEGREES_LEFT - getCurrentDegrees();
                else if (getCurrentDegrees() + offset > MAX_DEGREES_RIGHT)
                    offset = MAX_DEGREES_RIGHT - getCurrentDegrees();


                double apriltagOutput = apriltagPID.calculate(offset, 0);
                double ff = Math.abs(apriltagPID.getPositionError()) > 1 ? apriltag_kS * Math.signum(apriltagOutput) : 0;

                setPower(apriltagOutput + ff);
                break;

            default:
                setPower(0.0);
        }

        telemetry.addLine("----turret----");
        telemetry.addData("state", state);
        telemetry.addData("tagOffset [tx]", offsetFromTag);
    }
}
