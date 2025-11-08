package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.LatestGoalData;
import org.firstinspires.ftc.teamcode.ShooterState;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;

    private ShooterState state = ShooterState.IDLE;

    private final PIDFController velocityPID;

    public static double kP = 30.0;
    public static double kI = 5.0;
    public static double kD = 0.0;
    public static double kF = 0.8;

    public static double MAX_HOOD_ANGLE = 0.8;
    public static double MIN_HOOD_ANGLE = 0;

    public static double VELOCITY_TOLERANCE = 5.0;

    public static double INTAKE_VELOCITY = -1000;

    //Interpolation tables for hood and shooter speed
    private final InterpLUT speedTable = new InterpLUT();
    private final InterpLUT hoodTable = new InterpLUT();

    //Make sure interpolation table values have a big enough range to not throw out of bounds errors
    private void generateInterpolationTables() {
    }

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        velocityPID = new PIDFController(kP, kI, kD, kF);
        generateInterpolationTables();
    }

    public void setState(ShooterState newState) {
        state = newState;
    }

    public ShooterState getState() {
        return state;
    }

    //A percentage between 0.0 and 1.0 inclusive
    private void setHoodPercentage(double percentage) {
        hoodServo.setPosition(((MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * percentage) + MIN_HOOD_ANGLE);
    }

    private void setVelocity(double targetVelocity) {
        shooterMotor.setVelocity(targetVelocity);
    }

    public boolean atVelocity() {
        return Math.abs(getVelocity() - velocityPID.getSetPoint()) < VELOCITY_TOLERANCE;
    }

    //Returns the shooter velocity in ticks per second
    private double getVelocity() {
        return shooterMotor.getVelocity();
    }

    @Override
    public void periodic() {
        telemetry.addLine("----shooter----");
        telemetry.addData("state", state);
        telemetry.addData("velocity", getVelocity());

        //State-machine
        if (state == ShooterState.SHOOT) {
            velocityPID.setSetPoint(speedTable.get(LatestGoalData.getDistanceFromGoal()));
            setVelocity(velocityPID.calculate(getVelocity()));

            setHoodPercentage(hoodTable.get(LatestGoalData.getDistanceFromGoal()));
        }
        else if (state == ShooterState.INTAKE) {
            velocityPID.setSetPoint(INTAKE_VELOCITY);
            setVelocity(velocityPID.calculate(getVelocity()));

            setHoodPercentage(0.0); //Hood fully down to help balls intake
        }
        else {
            setVelocity(0.0);
        }
    }
}
