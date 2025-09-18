package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.stealthrobotics.library.StealthSubsystem;

public class TurretSubsystem extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final PIDController pid;
    private static double kP = 0;
    private static double kI = 0;
    private static double kD = 0;
    private static double tolerance = 1;
    public TurretSubsystem(HardwareMap hardwareMap){
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        pid = new PIDController(kP,kI,kD);
        pid.setTolerance(tolerance);
    }
    public void setPosition(double ticks){
        pid.setSetPoint(ticks);
    }
    public void setPower(double power){
        turretMotor.setPower(power);
    }
    public double getPosition(){
        return turretMotor.getCurrentPosition();
    }
    public double pidCalc(){
        return pid.calculate(getPosition());
    }
    public void resetEncoder(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
