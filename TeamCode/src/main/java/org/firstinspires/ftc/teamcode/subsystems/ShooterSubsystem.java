package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PoseTracker;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;

    public static double kP = 30.0;
    public static double kI = 5.0;
    public static double kD = 0.0;
    public static double kF = 12;

    public static double MAX_HOOD_ANGLE = 0.8;
    public static double MIN_HOOD_ANGLE = 0;

    private final PIDFController velocityPID;
    public static double VELOCITY_TOLERANCE = 10;

    //Interpolation tables for hood and shooter speed
    private final InterpLUT speedTable = new InterpLUT();
    private final InterpLUT hoodTable = new InterpLUT();

    // ? Tracks whether the shooter should spin to calculated velocity
    private boolean spinUp = false;

    private void generateInterpolationTables() {
        try {
            speedTable.add(0, 0.4);
            speedTable.add(20, 0.5);
            speedTable.add(80, 0.65);
            speedTable.add(124, 0.78);
            speedTable.add(200, 0.9);
            speedTable.add(2000, 1.0);

            speedTable.createLUT();

            hoodTable.add(0, 0);
            hoodTable.add(20, 0.3);
            hoodTable.add(80, 0.92);
            hoodTable.add(124, 1.0);
            hoodTable.add(2000, 1.0);

            hoodTable.createLUT();
        } catch (Exception ignored) {}
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

    public Command toggleSpinning() {
        return this.runOnce(() -> spinUp = !spinUp);
    }

    public Command stop() {
        return this.runOnce(() -> spinUp = false);
    }

    //Returns the shooter velocity in ticks per second
    private double getVelocity() {
        return shooterMotor.getVelocity();
    }


    @Override
    public void periodic() {
        telemetry.addLine("-----shooter-----");
        telemetry.addData("shooter power", shooterMotor.getPower());
        telemetry.addData("spinUp", spinUp);

        double distanceFromGoal = PoseTracker.getDistanceFromGoal();
        if (spinUp) {
            try {
                shooterMotor.setPower(speedTable.get(distanceFromGoal));
            }
            catch (Exception ignored) {};
        }
        else {
            setVelocity(0.0);
        }

        try {
            setHoodPercentage(hoodTable.get(distanceFromGoal));
        }
        catch (Exception ignored) {};
    }
}
