package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public static double veloSet = 0.0;
    public static double hoodSet = 0.0;

    private final PIDFController velocityPID;

    public static double kP = 0.02;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    private final double MAX_HOOD_ANGLE = 1;
    private final double MIN_HOOD_ANGLE = 0.15;

    public static double VELOCITY_TOLERANCE = 20.0; //In ticks per second

    public static double hoodAngle = 0.0;

    //Interpolation tables for hood and shooter speed
    private final InterpLUT speedTable = new InterpLUT();
    private final InterpLUT hoodTable = new InterpLUT();

    //Make sure interpolation table values have a big enough range to not throw out of bounds errors
    private void generateInterpolationTables() {
        speedTable.add(0, 1100);
        speedTable.add(21, 1100);
        speedTable.add(27, 1100);
        speedTable.add(31, 1200);
        speedTable.add(39.6, 1200);
        speedTable.add(60, 1300);
        speedTable.add(93, 1600);
        speedTable.add(100, 1600);
        speedTable.createLUT();

        hoodTable.add(0, 0.48);
        hoodTable.add(21, 0.48);
        hoodTable.add(27, 0.53);
        hoodTable.add(31, 0.55);
        hoodTable.add(39.6, 0.73);
        hoodTable.add(60, 0.8);
        hoodTable.add(93, 1.0);
        hoodTable.add(100, 1.0);
        hoodTable.createLUT();
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
        //State-machine
        telemetry.addData("clamped distance", MathFunctions.clamp(LatestGoalData.getDistanceFromGoal(), 21, 93));
        if (state == ShooterState.SHOOT) {

            velocityPID.setSetPoint(speedTable.get(MathFunctions.clamp(LatestGoalData.getDistanceFromGoal(), 21, 93)));
            setPower(velocityPID.calculate(getVelocity()));

            setHoodPercentage(hoodTable.get(MathFunctions.clamp(LatestGoalData.getDistanceFromGoal(), 21, 93)));
        }
        else {
            setHoodPercentage(0.0);
            setPower(0.0);
        }

        telemetry.addLine("----shooter----");
        telemetry.addData("state", state);
        telemetry.addData("hood pos", hoodServo.getPosition());
        telemetry.addData("velocity", getVelocity());
    }
}
