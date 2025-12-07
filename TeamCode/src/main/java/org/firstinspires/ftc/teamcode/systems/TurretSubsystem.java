package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
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

    private double constantAutoAngle;

    private double encoderOffset = 0.0;

    private TurretState state = TurretState.IDLE;

    private double offsetFromTag = 0.0;

    public static double ODO_KS_THRESHOLD = 3;

    public static double odo_kP = 0.01;
    public static double odo_kI = 0.0;
    public static double odo_kD = 0.0;
    public static double odo_kS = 0.2;

    public static double apriltag_kP = 0.01;
    public static double apriltag_kI = 10;
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

        if (Alliance.isBlue()) constantAutoAngle = -122;
        else constantAutoAngle = 122;

        resetEncoder();
    }

    public double getOffsetFromTag() {
        return offsetFromTag;
    }

    public Command setState(TurretState newState) {
        return new InstantCommand(() -> state = newState);
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

    public void switchToConstant() {
        state = TurretState.CONSTANT;
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
        switch (state) {
            case HOME:
                double odoOutput = odoPID.calculate(getCurrentDegrees(), 0);
                if (Math.abs(odoPID.getPositionError()) > ODO_KS_THRESHOLD) {
                    setPower(odoOutput + (odo_kS * Math.signum(odoPID.getPositionError())));
                }
                else setPower(odoOutput);
                break;

            case OBELISK:
                odoOutput = odoPID.calculate(getCurrentDegrees(), MathFunctions.clamp(poseEstimator.getObeliskTurretTargetAngle(), MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
                if (Math.abs(odoPID.getPositionError()) > ODO_KS_THRESHOLD) {
                    setPower(odoOutput + (odo_kS * Math.signum(odoPID.getPositionError())));
                }
                else setPower(odoOutput);
                break;

            case CONSTANT:
                odoOutput = odoPID.calculate(getCurrentDegrees(), MathFunctions.clamp(constantAutoAngle, MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
                if (Math.abs(odoPID.getPositionError()) > ODO_KS_THRESHOLD) {
                    setPower(odoOutput + (odo_kS * Math.signum(odoPID.getPositionError())));
                }
                else setPower(odoOutput);
                break;

            case ODOMETRY:
                double constantOffset = 0;
                if (poseEstimator.getRobotPose().getY() < 48) {
                    constantOffset = -7.5;
                }
                odoOutput = odoPID.calculate(getCurrentDegrees(), MathFunctions.clamp(poseEstimator.getTurretTargetAngle() + constantOffset, MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
                if (Math.abs(odoPID.getPositionError()) > ODO_KS_THRESHOLD) {
                    setPower(odoOutput + (odo_kS * Math.signum(odoPID.getPositionError())));
                }
                else setPower(odoOutput);
                break;

            case APRILTAG:
                double offset = -offsetFromTag;

                MathFunctions.clamp(getCurrentDegrees() + offset, MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT);
                if (getCurrentDegrees() + offset < MAX_DEGREES_LEFT)
                    offset = MAX_DEGREES_LEFT - getCurrentDegrees();
                else if (getCurrentDegrees() + offset > MAX_DEGREES_RIGHT)
                    offset = MAX_DEGREES_RIGHT - getCurrentDegrees();


                double apriltagOutput = apriltagPID.calculate(offset, 0);
                double ff = apriltag_kS * Math.signum(apriltagPID.getPositionError());

                setPower(apriltagOutput + ff);
                break;

            default:
                setPower(0.0);
        }

        apriltagPID.setPID(apriltag_kP, apriltag_kI, apriltag_kD);
    }
}
