package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor;
    private final ServoEx hoodServo;

    private final PIDController velocityPID; //In ticks per second
    private final double VELO_TOLERANCE = 0.0;

    public static double kP = 0.0;
    public static double kI = 0.0;
    public static double kD = 0.0;

    //Interpolation tables for hood and shooter speed
    private final InterpLUT speedTable = new InterpLUT();
    private final InterpLUT hoodTable = new InterpLUT();

    private void generateInterpolationTables() {
//        speedTable.add();
        speedTable.createLUT();
        hoodTable.createLUT();
    }

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        hoodServo = hardwareMap.get(ServoEx.class, "hoodServo");

        velocityPID = new PIDController(kP, kI, kD);
        velocityPID.setTolerance(VELO_TOLERANCE);

        generateInterpolationTables();
    }

    public void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public boolean atVelocity() {
        return velocityPID.atSetPoint();
    }

    //Spin up the motor to target velocity and then finish
    // ! native getVelocity method might cause issues because it doesn't update when motor power is 0
    public Command spinUp(double targetVelo) {
        return this.runOnce(() -> velocityPID.setSetPoint(targetVelo))
                .andThen(run(() -> shooterMotor.setPower(velocityPID.calculate(shooterMotor.getVelocity()))).interruptOn(this::atVelocity));
    }

    //Stop the motor
    public Command stop() {
        return this.runOnce(() -> shooterMotor.setPower(0));
    }
}
