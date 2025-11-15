package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import static java.lang.Math.pow;
import static java.lang.Math.sqrt;

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

    public static double fakeDistanceFromGoal = 0.0;

    private final double angleP = 0.03;
    private final double angleI = 0.05;
    private final double angleD = 0.0;

    private final double TICKS_PER_REVOLUTION = 1538; // (output ratio) * PPR = 4 * 384.5

    private final double MAX_DEGREES_RIGHT = 170;
    private final double MAX_DEGREES_LEFT = -170;

    private final Pose BLUE_GOAL_POSE = new Pose(16.3575, 130.3727);
    private final Pose RED_GOAL_POSE = new Pose(127.6425, 130.3727);

    private final Pose OBELISK_POSE = new Pose(72, 144);

    public TurretSubsystem(HardwareMap hardwareMap, PoseSupplier poseSupplier) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        this.poseSupplier = poseSupplier;

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
        return (getCurrentTicks() / TICKS_PER_REVOLUTION) * 360;
    }

    @Override
    public void periodic() {
        Pose2D robotPose = poseSupplier.getAsPose();
        Pose robotPosePedro = new Pose(robotPose.getX(DistanceUnit.INCH), robotPose.getY(DistanceUnit.INCH), robotPose.getHeading(AngleUnit.DEGREES));

//        double distanceFromGoal = fakeDistanceFromGoal;
//        telemetry.addData("LUTOutput", offsetTable.get(MathFunctions.clamp(distanceFromGoal, 0.25, 200)));
        double distanceFromGoal = sqrt(pow((robotPosePedro.getX() - goalPose.getX()), 2) + pow((robotPosePedro.getY() - goalPose.getY()), 2));
        LatestGoalData.updateDistanceFromGoal(distanceFromGoal);

        if (state == TurretState.GOAL) {
            double targetAngleDegrees = AngleUnit.RADIANS.toDegrees(Math.atan2(goalPose.getY() - robotPosePedro.getY(), goalPose.getX() - robotPosePedro.getX()));
            double turretTarget = AngleUnit.normalizeDegrees(robotPosePedro.getHeading() - targetAngleDegrees);

            turretTarget += offsetTable.get(MathFunctions.clamp(distanceFromGoal, 0.25, 200));

            trackingPID.setSetPoint(MathFunctions.clamp(turretTarget, MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
            setPower(trackingPID.calculate(getCurrentDegrees()));
        }
        else if (state == TurretState.OBELISK) {
            double targetAngleDegrees = AngleUnit.RADIANS.toDegrees(Math.atan2(OBELISK_POSE.getY() - robotPosePedro.getY(), OBELISK_POSE.getX() - robotPosePedro.getX()));
            double turretTarget = AngleUnit.normalizeDegrees(robotPosePedro.getHeading() - targetAngleDegrees);

            turretTarget += offsetTable.get(MathFunctions.clamp(distanceFromGoal, 0.25, 200));

            trackingPID.setSetPoint(MathFunctions.clamp(turretTarget, MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT));
            setPower(trackingPID.calculate(getCurrentDegrees()));
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
