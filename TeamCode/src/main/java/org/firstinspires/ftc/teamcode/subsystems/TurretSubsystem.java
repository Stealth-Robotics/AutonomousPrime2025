package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import org.stealthrobotics.library.StealthSubsystem;

public class TurretSubsystem extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final PIDController pid;
    private final PIDController tickPID;

    private final double POSITION_TOLERANCE_DEGREES = 1.0; //TODO tune tolerance
    private final double POSITION_TOLERANCE_TICKS = 10; //TODO tune tolerance

    private final double TICKS_PER_REVOLUTION = 1538; // (output ratio) * PPR = 4 * 384.5

    private final double MAX_ROTATION_WRAP = 5.0; //TODO tune to not rip wires out
    private final double WRAP_TOLERANCE = 0.25;

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        pid = new PIDController(0.0, 0.0, 0.0);
        tickPID = new PIDController(0.0, 0.0, 0.0);

        pid.setTolerance(POSITION_TOLERANCE_DEGREES);
        tickPID.setTolerance(POSITION_TOLERANCE_TICKS);
    }

    //Unwrap the turret rotation once
    public Command unwrap() {
        return new CommandBase() {
            @Override
            public void initialize() {
                if (getCurrentWraps() > 0)
                    pid.setSetPoint(turretMotor.getCurrentPosition() - TICKS_PER_REVOLUTION);
                else
                    pid.setSetPoint(turretMotor.getCurrentPosition() + TICKS_PER_REVOLUTION);
            }

            @Override
            public void execute() {
                setPower(tickPID.calculate(turretMotor.getCurrentPosition()));
            }

            @Override
            public boolean isFinished() {
                return pid.atSetPoint();
            }
        };
    }

//    public Command unWrapFully() {
//    }

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
        if (Math.abs(getCurrentWraps() - MAX_ROTATION_WRAP) < WRAP_TOLERANCE) {
            unwrap().schedule(); // ! Idk if this works
        }

        telemetry.addLine("----------turret----------");
        telemetry.addData("angle: ", getPosition());
    }
}
