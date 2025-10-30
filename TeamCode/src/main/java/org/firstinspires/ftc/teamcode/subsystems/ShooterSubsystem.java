package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.PoseTracker;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@Config
public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor;
    private final ServoEx hoodServo;

    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    //TODO: Tune
    public static double MAX_HOOD_ANGLE = 1;
    public static double MIN_HOOD_ANGLE = 0;

    private final PIDController velocityPID;
    public static double VELOCITY_TOLERANCE = 5.0;

    //Interpolation tables for hood and shooter speed
    private final InterpLUT speedTable = new InterpLUT();
    private final InterpLUT hoodTable = new InterpLUT();

    // ? Tracks whether the shooter should spin to calculated velocity
    private boolean spinUp = false;

    private double lastTime;
    private int lastTicks;

    private void generateInterpolationTables() {
        //Shooter speed
//        speedTable.add();
//        speedTable.createLUT();
//        hoodTable.createLUT();
    }

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        hoodServo = new SimpleServo(hardwareMap, "hoodServo", MIN_HOOD_ANGLE, MAX_HOOD_ANGLE);

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        velocityPID = new PIDController(kP, kI, kD);
        lastTime = (double) System.nanoTime() / 1E9;

        generateInterpolationTables();
    }

    //[0.0, 1.0]
    public void setHoodPercentage(double percentage) {
        hoodServo.turnToAngle(((MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * percentage) + MIN_HOOD_ANGLE);
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
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
        double time = (double) System.nanoTime() / 1E9;
        int ticks = shooterMotor.getCurrentPosition();
        double velo = (ticks - lastTicks) / (time - lastTime);
        lastTime = time;
        lastTicks = ticks;
        return velo;
    }

    @Override
    public void periodic() {
        telemetry.addLine("-----shooter-----");
        telemetry.addData("velo", getVelocity());
        telemetry.addData("atVelo", atVelocity());
        telemetry.addData("targetVelo", velocityPID.getSetPoint());

        if (spinUp) {
            velocityPID.setSetPoint(speedTable.get(PoseTracker.getDistanceFromGoal()));
            setPower(velocityPID.calculate(getVelocity()));
        }
        else {
            velocityPID.setSetPoint(0.0);
            setPower(velocityPID.calculate(getVelocity()));
        }
    }
}
