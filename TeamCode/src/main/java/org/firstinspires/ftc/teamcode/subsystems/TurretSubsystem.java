package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.LatestGoalData;
import org.firstinspires.ftc.teamcode.TurretState;
import org.stealthrobotics.library.StealthSubsystem;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class TurretSubsystem extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final PIDFController trackingPID;

    private TurretState state = TurretState.TARGET;

    public static double kP = 0.06;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.0;

    public static double TRACKING_TOLERANCE_DEGREES = 0.5;

    private final double TICKS_PER_REVOLUTION = 1538; // (output ratio) * PPR = 4 * 384.5
    private final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

    //Constraints for turret wiring based off of home position (0 ticks)
    private final double MAX_TICKS_RIGHT = 760;
    private final double MAX_TICKS_LEFT = -760;

    private boolean searchingRight = true;

    public TurretSubsystem(HardwareMap hardwareMap, boolean isAutonomous) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        trackingPID = new PIDFController(kP, kI, kD, kF);
        trackingPID.setTolerance(TRACKING_TOLERANCE_DEGREES);

        if (isAutonomous) resetEncoder();
    }

    public void setState(TurretState newState) {
        state = newState;
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

    private int getCurrentTicks() {
        return turretMotor.getCurrentPosition();
    }

    @Override
    public void periodic() {
        if (state == TurretState.SEARCH) {
            //Sweep for full range of motion searching for goal apriltag
//            if (pid.atSetPoint()) {
//                searchingRight = !searchingRight;
//                if (searchingRight) pid.setSetPoint(MAX_TICKS_RIGHT);
//                else pid.setSetPoint(MAX_TICKS_LEFT);
//            }
//            //Sweep slower
//            else setPower(MathFunctions.clamp(pid.calculate(getCurrentTicks()), -0.4, 0.4));
        }
        else if (state == TurretState.TARGET) {
            if (LatestGoalData.canSeeTag()) {
                trackingPID.setSetPoint(0); //Maybe add offsets for different alliances for better shots
                setPower(MathFunctions.clamp(trackingPID.calculate(-LatestGoalData.getHeadingOffsetFromGoal()), -0.34, 0.34));
            }
        }
        else if (state == TurretState.HOME) {
//            pid.setSetPoint(0); //Home turret
//            setPower(pid.calculate(getCurrentTicks()));
//            if (pid.atSetPoint()) {
//                setState(TurretState.IDLE);
//            }
        }
        else {
            setPower(0.0);
        }

        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("targetAngle", trackingPID.getSetPoint());
        dashboardTelemetry.addData("currentAngleFromGoal", LatestGoalData.getHeadingOffsetFromGoal());
        dashboardTelemetry.update();

        telemetry.addLine("----turret----");
        telemetry.addData("state", state);
        telemetry.addData("encoder ticks", getCurrentTicks());

        trackingPID.setPIDF(kP, kI, kD, kF);
    }
}
