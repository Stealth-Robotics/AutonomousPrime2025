package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.enums.TurretState;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.CoordinateInterpolationTable;
import org.stealthrobotics.library.StealthSubsystem;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class TurretSubsystem extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final PIDController trackingPID;

    private final PoseEstimator poseEstimator;

    private double encoderOffset = 0.0;

    private TurretState state = TurretState.IDLE;

    //The amount to aim to the right/left of the goal depending on where you are on the field
    private final CoordinateInterpolationTable offsetTable = new CoordinateInterpolationTable(2.5);

    public static double kP = 0.01;
    public static double kI = 0.025;
    public static double kD = 0.0;
    public static double kS = 0.18;

    private final double TICKS_PER_REVOLUTION = 4 * 537.7; // (output ratio) * PPR = 4 * 537.7

    private final double MAX_DEGREES_RIGHT = 170;
    private final double MAX_DEGREES_LEFT = -170;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        trackingPID = new PIDController(kP, kI, kD);

        poseEstimator = PoseEstimator.getInstance();

        setupLUT();
        resetEncoder();
    }

    //Turret offsets based on distance from goal
    private void setupLUT() {
        if (Alliance.isBlue()) {
            offsetTable.addPoint(29, 129, 30);
            offsetTable.addPoint(52, 106, 0);
            offsetTable.addPoint(72, 72, 0);
            offsetTable.addPoint(63, 5, -10);
            offsetTable.addPoint(105, 3, -15);
            offsetTable.addPoint(110, 83, -15);
            offsetTable.addPoint(83, 127, -10);
            offsetTable.addPoint(45, 82, 0);
        }
        else {
        }
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

    @Override
    public void periodic() {
        if (state == TurretState.TARGET) {
            double distanceFromGoal = poseEstimator.getDistanceFromGoal();
            double turretTarget = poseEstimator.getTurretTargetAngle();

            Pose robotPose = poseEstimator.getRobotPose();
            double offset = offsetTable.get(robotPose.getX(), robotPose.getY());

            telemetry.addData("currentOffset", offset);

            turretTarget += offset;
            turretTarget = MathFunctions.clamp(turretTarget, MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT);

            double pidOutput = trackingPID.calculate(getCurrentDegrees(), turretTarget);
            double staticFrictionCompensation = 0;

            if (Math.abs(trackingPID.getPositionError()) > 3)
                staticFrictionCompensation =  kS * Math.signum(trackingPID.getPositionError());

            setPower(pidOutput + staticFrictionCompensation);
        }
        else {
            //Stop all turret movement
            setPower(0.0);
        }

        telemetry.addLine("----turret----");
        telemetry.addData("state", state);
        telemetry.addData("error", trackingPID.getPositionError());
        telemetry.addData("at setpoint", trackingPID.atSetPoint());
    }
}
