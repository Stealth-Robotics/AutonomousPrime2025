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
import org.stealthrobotics.library.SquIDController;
import org.stealthrobotics.library.StealthSubsystem;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class TurretSubsystem extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final PIDController pid;

    private final PoseEstimator poseEstimator;

    private double encoderOffset = 0.0;

    private TurretState state = TurretState.IDLE;

    private double offsetFromTag = 0.0;

    public static double kP = 0.04;
    public static double kI = 0.05;
    public static double kD = 0.0;
    public static double kS = 0.1;

    private final double TICKS_PER_REVOLUTION = 4 * 537.7; // (output ratio) * PPR = 4 * 537.7

    private final double MAX_DEGREES_RIGHT = 170;
    private final double MAX_DEGREES_LEFT = -170;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        poseEstimator = PoseEstimator.getInstance();

        pid = new PIDController(kP, kI, kD);

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

    public void switchToOdometryControl() {
        state = TurretState.ODOMETRY;
        pid.reset();
    }

    public void switchToApriltagControl() {
        state = TurretState.APRILTAG;
        pid.reset();
    }

    public void updateOffsetFromTag(double newOffset) {
        offsetFromTag = -newOffset;
    }

    @Override
    public void periodic() {
        switch (state) {
            case ODOMETRY:
                double odoOutput = pid.calculate(getCurrentDegrees(), MathFunctions.clamp(poseEstimator.getTurretTargetAngle(), MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
                setPower(odoOutput + (kS * Math.signum(pid.getPositionError())));
                break;

            case APRILTAG:
                double apriltagOutput = pid.calculate(getCurrentDegrees(), MathFunctions.clamp(getCurrentDegrees() - offsetFromTag, MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
                setPower(apriltagOutput + (kS * Math.signum(pid.getPositionError())));
                break;

            default:
                setPower(0.0);
        }

        pid.setPID(kP, kI, kD);

        telemetry.addLine("----turret----");
        telemetry.addData("tagOffset [tx]", offsetFromTag);
        telemetry.addData("state", state);
    }
}
