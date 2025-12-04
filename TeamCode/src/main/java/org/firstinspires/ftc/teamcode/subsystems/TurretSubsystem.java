package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.enums.TurretState;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.ArrayList;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class TurretSubsystem extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final PIDController trackingPID;

    private final PoseEstimator poseEstimator;

    private double encoderOffset = 0.0;

    private TurretState state = TurretState.IDLE;

    //The amount to aim to the right/left of the goal depending on where you are on the field
    private final ArrayList<CoordinateValue> pointOffsetValues = new ArrayList<>();

    public static double kP = 0.05;
    public static double kI = 0.05;
    public static double kD = 0.0;

    private final double TICKS_PER_REVOLUTION = 4 * 537.7; // (output ratio) * PPR = 4 * 537.7

    private final double MAX_DEGREES_RIGHT = 170;
    private final double MAX_DEGREES_LEFT = -170;

    private static class CoordinateValue {
        public final double x;
        public final double y;
        public final double value;

        public CoordinateValue(double x, double y, double value) {
            this.x = x;
            this.y = y;
            this.value = value;
        }
    }

    public TurretSubsystem(HardwareMap hardwareMap) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        trackingPID = new PIDController(kP, kI, kD);

        poseEstimator = PoseEstimator.getInstance();

        setupLUT();
        resetEncoder();
    }

    //Turret offsets based on distance from goal
    private void setupLUT() {
        if (Alliance.isBlue()) {
            pointOffsetValues.add(new CoordinateValue(72.01059897114912, 72.01038270484744, 0.0));
            pointOffsetValues.add(new CoordinateValue(72.01059897114912, 72.01038270484744, 0.0));
            pointOffsetValues.add(new CoordinateValue(73.99875911202018, 71.99345145638533, 0.0));
            pointOffsetValues.add(new CoordinateValue(73.90794168307087, 75.17712210107038, 0.0));
            pointOffsetValues.add(new CoordinateValue(73.2325804702879, 75.16592431256151, 0.0));
            pointOffsetValues.add(new CoordinateValue(72.51838744156004, 70.47945758489173, 0.0));
            pointOffsetValues.add(new CoordinateValue(67.72246593565453, 70.72949026513287, -8.0));
            pointOffsetValues.add(new CoordinateValue(38.71174489419291, 94.8746520515502, -8.0));
            pointOffsetValues.add(new CoordinateValue(41.13675335260827, 116.66567344365157, -8.0));
            pointOffsetValues.add(new CoordinateValue(102.38134188914863, 13.144757128137304, -8.0));
            pointOffsetValues.add(new CoordinateValue(85.32074695497047, 12.751640259750246, -6.0));
            pointOffsetValues.add(new CoordinateValue(70.19311619555856, 11.399209330401083, -3.0));
            pointOffsetValues.add(new CoordinateValue(69.63153028112697, 28.7363463874877, -3.0));
            pointOffsetValues.add(new CoordinateValue(116.64253775529036, 92.13058832123525, -5.0));
            pointOffsetValues.add(new CoordinateValue(87.44645246370573, 134.4695708507628, -11.0));
            pointOffsetValues.add(new CoordinateValue(44.94646207554134, 131.4275786632628, -4.0));
            pointOffsetValues.add(new CoordinateValue(20.203809362696852, 113.78378560223918, -29.0));
            pointOffsetValues.add(new CoordinateValue(32.14357391117126, 118.1648602823573, -11.0));
            pointOffsetValues.add(new CoordinateValue(61.809586652620574, 77.90446123738928, -11.0));
            pointOffsetValues.add(new CoordinateValue(86.7724224901575, 95.88142455093505, -6.0));
            pointOffsetValues.add(new CoordinateValue(42.89465235912893, 8.548886757197343, -2.0));
            pointOffsetValues.add(new CoordinateValue(59.95049904650591, 7.311492679625985, -8.0));
            pointOffsetValues.add(new CoordinateValue(78.29260638379675, 8.26933673047644, -8.0));
            pointOffsetValues.add(new CoordinateValue(77.0625653604823, 18.529804860513043, -8.0));
        }
        else {
        }
    }

    public Command setState(TurretState newState) {
        return this.runOnce(() -> state = newState);
    }

    public TurretState getState() {
        return state;
    }

    private void resetEncoder() {
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setPower(double power) {
        turretMotor.setPower(power);
    }

    //Essentially re-zeros the motor encoder without homing
    public void setEncoderOffset(double encoderOffset) {
        this.encoderOffset = encoderOffset;
    }

    //Returns the raw, unaltered ticks of the motor
    public int getRawTicks() {
        return turretMotor.getCurrentPosition();
    }

    //Returns the ticks based off of where 0 should be
    private double getCurrentTicks() {
        return turretMotor.getCurrentPosition() + encoderOffset;
    }

    private double getCurrentDegrees() {
        return (getCurrentTicks() / TICKS_PER_REVOLUTION) * 360;
    }

    private double getNearestOffset(Pose robotPose) {
        CoordinateValue nearest = null;
        double minDistance = Double.MAX_VALUE;

        for (CoordinateValue value : pointOffsetValues) {
            double distance = Math.sqrt(Math.pow(robotPose.getX() - value.x, 2) + Math.pow(robotPose.getY() - value.y, 2));
            if (distance < minDistance) {
                minDistance = distance;
                nearest = value;
            }
        }

        return nearest.value;
    }

    @Override
    public void periodic() {
        if (state == TurretState.TARGET) {
            double distanceFromGoal = poseEstimator.getDistanceFromGoal();
            double turretTarget = poseEstimator.getTurretTargetAngle();

            Pose robotPose = poseEstimator.getRobotPose();
            double offset = 0;

            if (!pointOffsetValues.isEmpty()) {
               offset = getNearestOffset(robotPose);
            }

            turretTarget += offset;
            turretTarget = MathFunctions.clamp(turretTarget, MAX_DEGREES_LEFT, MAX_DEGREES_RIGHT);

            double pidOutput = trackingPID.calculate(getCurrentDegrees(), turretTarget);
            setPower(pidOutput);

            telemetry.addLine("----turret----");
            telemetry.addData("offset", offset);
        }
        else {
            //Stop all turret movement
            setPower(0.0);

            telemetry.addLine("----turret----");
        }

        telemetry.addData("state", state);
        telemetry.addData("error", trackingPID.getPositionError());
        telemetry.addData("at setpoint", trackingPID.atSetPoint());
    }
}
