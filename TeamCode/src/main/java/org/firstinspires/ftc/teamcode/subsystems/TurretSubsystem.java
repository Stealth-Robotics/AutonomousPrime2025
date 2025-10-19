package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import org.stealthrobotics.library.AnglePIDController;
import org.stealthrobotics.library.StealthSubsystem;

@Config
public class TurretSubsystem extends StealthSubsystem {
    private final DcMotorEx turretMotor;

    private final AnglePIDController pid;
    private final PIDController tickPID;

    public static double kP = 0.01;
    public static double kI = 0.0045;
    public static double kD = 0.0008;
    public static double kF = 0.0;

    public static double tickP = 0.008;
    public static double tickI = 0.005;
    public static double tickD = 0.002;

    public static double POSITION_TOLERANCE_DEGREES = 0.5;
    public static double POSITION_TOLERANCE_TICKS = 15;

    private final double TICKS_PER_REVOLUTION = 1538; // (output ratio) * PPR = 4 * 384.5

    private final double MAX_ROTATION_WRAP = 3.0; //TODO Set to actual value
    private final double WRAP_TOLERANCE = 0.1;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        pid = new AnglePIDController(kP, kI, kD, kF);
        tickPID = new PIDController(tickP, tickI, tickD);

        pid.setPositionTolerance(POSITION_TOLERANCE_DEGREES);
        tickPID.setTolerance(POSITION_TOLERANCE_TICKS);

        resetEncoder();
    }

    private void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Unwrap the turret rotation once
    public Command unwrap() {
        return new CommandBase() {
            @Override
            public void initialize() {
                if (getCurrentWraps() > 0)
                    tickPID.setSetPoint(turretMotor.getCurrentPosition() - TICKS_PER_REVOLUTION);
                else
                    tickPID.setSetPoint(turretMotor.getCurrentPosition() + TICKS_PER_REVOLUTION);
            }

            @Override
            public void execute() {
                setPower(tickPID.calculate(turretMotor.getCurrentPosition()));
            }

            @Override
            public boolean isFinished() {
                return tickPID.atSetPoint();
            }
        };
    }

    //Unwrap turret fully back to 0
    public Command unWrapFully() {
        return new CommandBase() {
            @Override
            public void initialize() {
                tickPID.setSetPoint(0);
            }

            @Override
            public void execute() {
                setPower(tickPID.calculate(turretMotor.getCurrentPosition()));
            }

            @Override
            public boolean isFinished() {
                return tickPID.atSetPoint();
            }
        };
    }

    //Turret angle in [0, 360)
    public double getPosition() {
        return wrapAngle((turretMotor.getCurrentPosition() / TICKS_PER_REVOLUTION) * 360);
    }

    //Get the wrap count (- or +)
    public double getCurrentWraps() {
        return turretMotor.getCurrentPosition() / TICKS_PER_REVOLUTION;
    }

    private double wrapAngle(double theta) {
        while (theta >= 360) theta -= 360;
        while (theta < 0) theta += 360;
        return theta;
    }

    public double pidCalculate() {
        return pid.calculate(getPosition());
    }

    public void setTarget(double target) {
        pid.setSetPoint(target);
    }

    public void setPower(double power) {
        turretMotor.setPower(power);
    }

    @Override
    public void periodic() {
        if (Math.abs(Math.abs(getCurrentWraps()) - Math.abs(MAX_ROTATION_WRAP)) < WRAP_TOLERANCE) {
            unWrapFully().schedule(); // ! unwrap partially or fully?
        }

        telemetry.addLine("----------turret----------");
        telemetry.addData("angle: ", getPosition());
        telemetry.addData("wraps: ", getCurrentWraps());
        telemetry.addData("ticks: ", turretMotor.getCurrentPosition());
    }
}
