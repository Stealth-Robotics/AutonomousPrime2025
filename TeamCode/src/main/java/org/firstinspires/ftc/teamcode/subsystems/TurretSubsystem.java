package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
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
import org.firstinspires.ftc.teamcode.TurretState;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class TurretSubsystem extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final PIDController trackingPID;

    private final DriveSubsystem drive;
    private final Pose goalPose;

    private TurretState state = TurretState.SEARCH;

    public static double tickP = 0.005;
    public static double tickI = 0.00;
    public static double tickD = 0.0;

    public static double angleP = 0.02;
    public static double angleI = 0.0;
    public static double angleD = 0.0;

    private final double TICKS_PER_REVOLUTION = 1538; // (output ratio) * PPR = 4 * 384.5
    private final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

    private final double MAX_TICKS_RIGHT = 760;
    private final double MAX_TICKS_LEFT = -760;

    private final double MAX_DEGREES_RIGHT = 170;
    private final double MAX_DEGREES_LEFT = -170;

    private final Pose BLUE_GOAL_POSE = new Pose(16.3575, 130.3727);
    private final Pose RED_GOAL_POSE = new Pose(127.6425, 130.3727);

    public TurretSubsystem(HardwareMap hardwareMap, DriveSubsystem drive, boolean isAutonomous) {
        this.drive = drive;
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        trackingPID = new PIDController(angleP, angleI, angleD);

        if (Alliance.get() == Alliance.BLUE) goalPose = BLUE_GOAL_POSE;
        else goalPose = RED_GOAL_POSE;

        if (isAutonomous) resetEncoder();
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

    private int getCurrentTicks() {
        return turretMotor.getCurrentPosition();
    }

    private double getCurrentDegrees() {
        return AngleUnit.normalizeDegrees((getCurrentTicks() / TICKS_PER_REVOLUTION) * 360);
    }

    @Override
    public void periodic() {
        if (state == TurretState.SEARCH) {
            if (!LatestGoalData.canSeeTag()) {
                Pose2D robotPose = drive.getPose();
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
                setPower(trackingPID.calculate(getCurrentTicks()));
            }
            else {
                trackingPID.setPID(angleP, angleI, angleD);
                trackingPID.reset();
                setState(TurretState.SEARCH);
            }
        }
        else {
            setPower(0.0);
        }

        telemetry.addLine("----turret----");
        telemetry.addData("state", state);
        telemetry.addData("ticks", getCurrentTicks());
        telemetry.addData("degrees", getCurrentDegrees());
    }
}
