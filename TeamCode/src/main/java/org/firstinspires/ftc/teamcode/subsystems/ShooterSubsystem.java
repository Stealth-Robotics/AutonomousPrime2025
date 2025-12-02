package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.enums.ShooterState;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class ShooterSubsystem extends StealthSubsystem {
    private final DcMotorEx shooterMotor;
    private final Servo hoodServo;

    private final PoseEstimator poseEstimator;

    private ShooterState state = ShooterState.IDLE;

    private final PIDFController velocityPID;

    public static double kP = 0.003;
    public static double kI = 0.3;
    public static double kD = 0.0;
    public static double kV = 0.0008;

    private final double MAX_HOOD_ANGLE = 1;
    private final double MIN_HOOD_ANGLE = 0.15;

    public static double VELOCITY_TOLERANCE = 50.0;

    //Interpolation tables for hood and shooter speed
    private final InterpLUT speedTable = new InterpLUT();
    private final InterpLUT hoodTable = new InterpLUT();

    public ShooterSubsystem(HardwareMap hardwareMap) {
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");
        hoodServo = hardwareMap.get(Servo.class, "hoodServo");

        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        velocityPID = new PIDFController(kP, kI, kD, kV);

        poseEstimator = PoseEstimator.getInstance();

        generateInterpolationTables();
    }

    //Make sure interpolation table values have a big enough range to not throw out of bounds errors
    private void generateInterpolationTables() {
        speedTable.add(0, 900);
        speedTable.add(15.36, 900);
        speedTable.add(55.06, 1100);
        speedTable.add(82.44, 1250);
        speedTable.add(131, 1520);
        speedTable.add(144.34, 1800);
        speedTable.createLUT();

        hoodTable.add(0, 0.25);
        hoodTable.add(15.36, 0.3);
        hoodTable.add(55.06, 0.6);
        hoodTable.add(82.44, 0.7);
        hoodTable.add(131, 1);
        hoodTable.add(144.34, 1);
        hoodTable.createLUT();
    }

    public Command setState(ShooterState newState) {
        return this.runOnce(() -> state = newState);
    }

    public ShooterState getState() {
        return state;
    }

    //A percentage between 0.0 and 1.0 inclusive
    private void setHoodPercentage(double percentage) {
        hoodServo.setPosition(((MAX_HOOD_ANGLE - MIN_HOOD_ANGLE) * percentage) + MIN_HOOD_ANGLE);
    }

    private void setPower(double power) {
        shooterMotor.setPower(power);
    }

    public boolean atVelocity() {
        return Math.abs(getVelocity() - velocityPID.getSetPoint()) < VELOCITY_TOLERANCE;
    }

    //Returns the shooter velocity in ticks per second
    private double getVelocity() {
        return -shooterMotor.getVelocity();
    }

    @Override
    public void periodic() {
        double distanceFromGoal = poseEstimator.getDistanceFromGoal();

        //Update hood angle based off of distance from the goal
        setHoodPercentage(hoodTable.get(MathFunctions.clamp(distanceFromGoal, 0.25, 144)));

        //State-machine
        if (state == ShooterState.SHOOT) {
            double velocitySetpoint = speedTable.get(MathFunctions.clamp(distanceFromGoal, 0.25, 144));
            velocityPID.setSetPoint(velocitySetpoint);
            setPower(velocityPID.calculate(getVelocity()));
        }
        else {
            velocityPID.setSetPoint(0);
            setPower(0.0);
        }

        telemetry.addLine("----shooter----");
        telemetry.addData("state", state);
        telemetry.addData("velocity", getVelocity());
        telemetry.addData("target", velocityPID.getSetPoint());
        telemetry.addData("at velocity", atVelocity());
        telemetry.addData("hood position", hoodServo.getPosition());
    }
}
