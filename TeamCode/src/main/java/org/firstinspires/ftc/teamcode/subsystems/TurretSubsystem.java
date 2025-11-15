package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.LatestGoalData;
import org.firstinspires.ftc.teamcode.PoseSupplier;
import org.firstinspires.ftc.teamcode.TurretState;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class TurretSubsystem extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final PIDController trackingPID;

    private final PoseSupplier poseSupplier;
    private final Pose goalPose;

    private double encoderOffset = 0.0;

    private TurretState state = TurretState.IDLE;

    private final double TURRET_TOLERANCE_TICKS = 5;

    public static double tickP = 0.004;
    public static double tickI = 0.00;
    public static double tickD = 0.0;

    //velo pid constants
    public static double kS = 0.0;
    public static double kV = 0.0;
    public static double kA = 0.0;

    public static double vP = 0.0;
    public static double vI = 0.0;
    public static double vD = 0.0;

    public static double angleP = 0.02;
    public static double angleI = 0.0;
    public static double angleD = 0.0;

    private final double TICKS_PER_REVOLUTION = 1538; // (output ratio) * PPR = 4 * 384.5
    private final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

//    private final double MAX_TICKS_RIGHT = 760;
//    private final double MAX_TICKS_LEFT = -760;
//
//    private final double MAX_DEGREES_RIGHT = 170;
//    private final double MAX_DEGREES_LEFT = -170;

    private final double MAX_TICKS_RIGHT = 200;
    private final double MAX_TICKS_LEFT = -200;

    private final double MAX_DEGREES_RIGHT = 90;
    private final double MAX_DEGREES_LEFT = -90;

    private final Pose BLUE_GOAL_POSE = new Pose(16.3575, 130.3727);
    private final Pose RED_GOAL_POSE = new Pose(127.6425, 130.3727);

    public TurretSubsystem(HardwareMap hardwareMap, PoseSupplier poseSupplier) {
        SimpleMotorFeedforward turretController = new SimpleMotorFeedforward(kS, kV, kA);
        PIDController velocityPIDController = new PIDController(vP, vI, vD);
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        this.poseSupplier = poseSupplier;

        trackingPID = new PIDController(angleP, angleI, angleD);
        trackingPID.setTolerance(TURRET_TOLERANCE_TICKS);

        if (Alliance.get() == Alliance.BLUE)
            goalPose = BLUE_GOAL_POSE;
        else
            goalPose = RED_GOAL_POSE;

        resetEncoder();
    }

    public void setState(TurretState newState) {
        state = newState;
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
        return AngleUnit.normalizeDegrees((getCurrentTicks() / TICKS_PER_REVOLUTION) * 360);
    }

    public void setSearching() {
        trackingPID.setPID(angleP, angleI, angleD);
        trackingPID.reset();
        setState(TurretState.SEARCH);
    }

    public double calculatePIDFVelocity(double tX){
        double pidOutput = trackingPID.calculate(tX, 0);

        double feedforwardOut = kS * Math.signum(pidOutput) + kV * pidOutput;

        return  pidOutput + feedforwardOut;
    }

    @Override
    public void periodic() {
        if (state == TurretState.SEARCH) {
            if (!LatestGoalData.canSeeTag()) {
                Pose2D robotPose = poseSupplier.getAsPose();
                Pose robotPosePedro = new Pose(robotPose.getX(DistanceUnit.INCH), robotPose.getY(DistanceUnit.INCH), robotPose.getHeading(AngleUnit.DEGREES));
                double targetAngleDegrees = AngleUnit.RADIANS.toDegrees(Math.atan2(goalPose.getY() - robotPosePedro.getY(), goalPose.getX() - robotPosePedro.getX()));
                trackingPID.setSetPoint(MathFunctions.clamp(AngleUnit.normalizeDegrees(robotPosePedro.getHeading() - targetAngleDegrees), MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
                setPower(trackingPID.calculate(getCurrentDegrees()));
            }
            else {
                trackingPID.setPID(tickP, tickI, tickD);
                trackingPID.reset();
                setState(TurretState.TARGET);
            }
        }
        else if (state == TurretState.TARGET) {
            if (LatestGoalData.canSeeTag()) {
                double targetTicks = MathFunctions.clamp(getCurrentTicks() + (LatestGoalData.getHeadingOffsetFromGoal() * TICKS_PER_DEGREE), MAX_TICKS_LEFT, MAX_TICKS_RIGHT);
                trackingPID.setSetPoint(targetTicks);

                double motorVeloOutput = calculatePIDFVelocity(LatestGoalData.getDistanceFromGoal());

                setPower(trackingPID.calculate(getCurrentTicks()));
            }
            else {
                trackingPID.setPID(angleP, angleI, angleD);
                trackingPID.reset();
                setState(TurretState.SEARCH);
            }
        }
        else if (state == TurretState.HOME) {
            //Set PID to HOME turret and then transition to HOMING to finish
            trackingPID.setPID(tickP, tickI, tickD);
            trackingPID.reset();

            //Center turret back to starting position (0 degrees, 0 ticks)
            trackingPID.setSetPoint(0);

            setState(TurretState.HOMING);
        }
        else if (state == TurretState.HOMING) {
            //Finish HOMING and then set state to IDLE
            setPower(trackingPID.calculate(getCurrentTicks()));

            if (trackingPID.atSetPoint())
                setState(TurretState.IDLE);
        }
        else {
            //Stop all turret movement
            setPower(0.0);
        }

        telemetry.addLine("----turret----");
        telemetry.addData("state", state);
        telemetry.addData("ticks", getCurrentTicks());
        telemetry.addData("degrees", getCurrentDegrees());
    }
}
