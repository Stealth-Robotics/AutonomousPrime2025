package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.ShooterState;
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

    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0.0;
    public static double kS = 0.74;

    private final double MAX_HOOD_ANGLE = 1;
    private final double MIN_HOOD_ANGLE = 0.15;

    public static double VELOCITY_TOLERANCE = 10.0; //TODO: Tune to a reasonable value

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
        speedTable.add(15, 1000);
        speedTable.add(25, 1100);
        speedTable.add(30, 1200);
        speedTable.add(40, 1300);
        speedTable.add(68, 1300);
        speedTable.add(120, 1550);
        speedTable.add(210, 2000);
        speedTable.createLUT();

        hoodTable.add(0, 0.35);
        hoodTable.add(15, 0.48);
        hoodTable.add(25, 0.53);
        hoodTable.add(30, 0.55);
        hoodTable.add(40, 0.73);
        hoodTable.add(68, 0.8);
        hoodTable.add(120, 1.0);
        hoodTable.add(210, 1.0);
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
        setHoodPercentage(hoodTable.get(MathFunctions.clamp(distanceFromGoal, 0.25, 200)));

        //State-machine
        if (state == ShooterState.SHOOT) {
            double velocitySetpoint = speedTable.get(MathFunctions.clamp(distanceFromGoal, 0.25, 200));
            velocityPID.setSetPoint(velocitySetpoint);
            setPower(velocityPID.calculate(getVelocity()) + (kS * Math.signum(velocitySetpoint)));
        }
        else {
            setPower(0.0);
        }

        telemetry.addLine("----shooter----");
        telemetry.addData("state", state);
        telemetry.addData("hood position", hoodServo.getPosition());
        telemetry.addData("velocity", getVelocity());
        telemetry.addData("at velocity", atVelocity());
    }
}
