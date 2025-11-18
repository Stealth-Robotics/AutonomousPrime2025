package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.drivebase.HDrive;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

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

    //The amount to aim to the right/left of the target as you get farther away (scales linearly)
    private final InterpLUT offsetTable = new InterpLUT();

    private final double TURRET_TOLERANCE_TICKS = 5;

    public static double tickP = 0.005;
    public static double tickI = 0.05;
    public static double tickD = 0.0008;
    public static double tickF = 0.005;

    public static double angleP = 0.03;
    public static double angleI = 0.05;
    public static double angleD = 0.0;

    private final double TICKS_PER_REVOLUTION = 1538; // (output ratio) * PPR = 4 * 384.5
    private final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

    private final double MAX_TICKS_RIGHT = 760;
    private final double MAX_TICKS_LEFT = -760;

    private final double MAX_DEGREES_RIGHT = 170;
    private final double MAX_DEGREES_LEFT = -170;

    private final Pose BLUE_GOAL_POSE = new Pose(16.3575, 130.3727);
    private final Pose RED_GOAL_POSE = new Pose(127.6425, 130.3727);

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        trackingPID = new PIDController(angleP, angleI, angleD);
        trackingPID.setTolerance(TURRET_TOLERANCE_TICKS);

        if (Alliance.get() == Alliance.BLUE)
            goalPose = BLUE_GOAL_POSE;
        else
            goalPose = RED_GOAL_POSE;

        setupLUT();
        resetEncoder();
    }

    //Setup turret offsets relative to goal distance
    private void setupLUT() {
        offsetTable.add(0, -8);
        offsetTable.add(210, -12);
        offsetTable.createLUT();
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

    public void setSearching() {
        trackingPID.setPID(angleP, angleI, angleD);
        setState(TurretState.SEARCH);
    }

    @Override
    public void periodic() {
        Pose2D robotPose = poseSupplier.getAsPose();
        Pose robotPosePedro = new Pose(robotPose.getX(DistanceUnit.INCH), robotPose.getY(DistanceUnit.INCH), robotPose.getHeading(AngleUnit.DEGREES));

        double distanceFromGoal = sqrt(pow((robotPosePedro.getX() - goalPose.getX()), 2) + pow((robotPosePedro.getY() - goalPose.getY()), 2));
        LatestGoalData.updateDistanceFromGoal(distanceFromGoal);

        if (state == TurretState.SEARCH) {
//            if (!LatestGoalData.canSeeTag()) {
//                Pose2D robotPose = poseSupplier.getAsPose();
//                Pose robotPosePedro = new Pose(robotPose.getX(DistanceUnit.INCH), robotPose.getY(DistanceUnit.INCH), robotPose.getHeading(AngleUnit.DEGREES));
//                double targetAngleDegrees = AngleUnit.RADIANS.toDegrees(Math.atan2(goalPose.getY() - robotPosePedro.getY(), goalPose.getX() - robotPosePedro.getX()));
//                trackingPID.setSetPoint(MathFunctions.clamp(AngleUnit.normalizeDegrees(robotPosePedro.getHeading() - targetAngleDegrees), MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
//                setPower(trackingPID.calculate(getCurrentDegrees()));
//            }
//            else {
//                trackingPID.setPID(tickP, tickI, tickD);
////                trackingPID.reset();
//                setState(TurretState.TARGET);
//            }

            double targetAngleDegrees = AngleUnit.RADIANS.toDegrees(Math.atan2(goalPose.getY() - robotPosePedro.getY(), goalPose.getX() - robotPosePedro.getX()));
            double turretTarget = AngleUnit.normalizeDegrees(robotPosePedro.getHeading() - targetAngleDegrees);

            turretTarget += offsetTable.get(MathFunctions.clamp(distanceFromGoal, 0.25, 200));

            trackingPID.setSetPoint(MathFunctions.clamp(turretTarget, MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
            setPower(trackingPID.calculate(getCurrentDegrees()));
        }
        else if (state == TurretState.TARGET) {
//            if (LatestGoalData.canSeeTag()) {
//                double targetTicks = MathFunctions.clamp(getCurrentTicks() + (LatestGoalData.getHeadingOffsetFromGoal() * TICKS_PER_DEGREE), MAX_TICKS_LEFT, MAX_TICKS_RIGHT);
//                trackingPID.setSetPoint(targetTicks);
//                double sign = -Math.signum(LatestGoalData.getHeadingOffsetFromGoal());
//                setPower(trackingPID.calculate(getCurrentTicks()) + (sign * tickF));
//            }
//            else {
//                trackingPID.setPID(angleP, angleI, angleD);
////                trackingPID.reset();
//                setState(TurretState.SEARCH);
//            }
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
