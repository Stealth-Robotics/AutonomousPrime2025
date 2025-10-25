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
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@Config
public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor;
    private final ServoEx hoodServo;

    public static double kP = 1.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    private final int MAX_VELOCITY = 2800;

    public static double MAX_HOOD_ANGLE = 0.85;
    public static double MIN_HOOD_ANGLE = 0.07;

    private final PIDController velocityPID;
    public static double VELO_TOLERANCE = 5.0;

    //Interpolation tables for hood and shooter speed
    private final InterpLUT speedTable = new InterpLUT();
    private final InterpLUT hoodTable = new InterpLUT();

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
        velocityPID.setTolerance(VELO_TOLERANCE);

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
        return velocityPID.atSetPoint();
    }

    //Spin up the motor to target velocity and then finish
    public Command spinUp(double velocityPercentage) {
        return this.runOnce(() -> velocityPID.setSetPoint(velocityPercentage * MAX_VELOCITY))
                .andThen(run(() -> shooterMotor.setPower(velocityPID.calculate(getVelocity()))).interruptOn(this::atVelocity));
    }

    //Stop the motor
    public Command stop() {
        return this.runOnce(() -> spinUp(0.0));
    }

    //Returns the shooter velocity in ticks per second
    private double getVelocity() {
        return shooterMotor.getVelocity();
    }

    @Override
    public void periodic() {
        telemetry.addLine("-----shooter-----");
        telemetry.addData("velo", getVelocity());
        telemetry.addData("atVelo", atVelocity());
        telemetry.addData("targetVelo", velocityPID.getSetPoint());
    }
}
