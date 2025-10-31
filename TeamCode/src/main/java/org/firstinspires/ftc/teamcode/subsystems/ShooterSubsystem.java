package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PoseTracker;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@Config
public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;

    public static double kP = 15.0;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.35;

    public static double MAX_HOOD_ANGLE = 0.8;
    public static double MIN_HOOD_ANGLE = 0;

    private final PIDFController velocityPID;
    public static double VELOCITY_TOLERANCE = 5.0;

    //Interpolation tables for hood and shooter speed
    private final InterpLUT speedTable = new InterpLUT();
    private final InterpLUT hoodTable = new InterpLUT();

    public static int testVelocity = 0;

    // ? Tracks whether the shooter should spin to calculated velocity
    private boolean spinUp = false;

    private void generateInterpolationTables() {
        //Shooter speed
//        speedTable.add();
//        speedTable.createLUT();
//        hoodTable.createLUT();
    }

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        velocityPID = new PIDFController(kP, kI, kD, kF);
        generateInterpolationTables();
    }

    //[0.0, 1.0]
    public void setHoodPercentage(double percentage) {
        hoodServo.setPosition(((MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * percentage) + MIN_HOOD_ANGLE);
    }

    public void setVelocity(double velo) {
        shooterMotor.setVelocity(velo);
    }

    public boolean atVelocity() {
        return Math.abs(getVelocity() - velocityPID.getSetPoint()) < VELOCITY_TOLERANCE;
    }

    public Command spinToVelocity() {
        return this.runOnce(() -> spinUp = true);
    }

    public Command stop() {
        return this.runOnce(() -> spinUp = false);
    }

    //Returns the shooter velocity in ticks per second
    private double getVelocity() {
        return shooterMotor.getVelocity();
    }

    //Convert rpms to ticks per second
    private double rpmToTPS(int rpm) {
        double rotationsPerSecond = rpm / 60.0;
        return 28 * rotationsPerSecond;
    }

    @Override
    public void periodic() {
        double currVelo = getVelocity();
        telemetry.addLine("-----shooter-----");
        telemetry.addData("velo", currVelo);
        telemetry.addData("atVelo", atVelocity());
        telemetry.addData("targetVelo", velocityPID.getSetPoint());

        // ! For graphing
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("target", velocityPID.getSetPoint());
        dashboardTelemetry.addData("current", currVelo);
        dashboardTelemetry.update();

//        if (spinUp) {
////            velocityPID.setSetPoint(speedTable.get(PoseTracker.getDistanceFromGoal()));
//            velocityPID.setSetPoint(rpmToTPS(testVelocity));
//            setVelocity(velocityPID.calculate(currVelo));
//        }
//        else {
//            velocityPID.setSetPoint(0.0);
//            setVelocity(velocityPID.calculate(currVelo));
//        }
//        shooterMotor.setPower(1.0);
    }
}
