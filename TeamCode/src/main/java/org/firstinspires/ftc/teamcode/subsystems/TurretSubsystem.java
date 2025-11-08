package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import org.firstinspires.ftc.teamcode.LatestGoalData;
import org.firstinspires.ftc.teamcode.TurretState;
import org.stealthrobotics.library.StealthSubsystem;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class TurretSubsystem extends StealthSubsystem {
    private final DcMotorEx turretMotor;
    private final PIDController pid;

    private TurretState state = TurretState.IDLE;

    public static double kP = 0.05;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double POSITION_TOLERANCE_TICKS = 15;

    private final double TICKS_PER_REVOLUTION = 1538; // (output ratio) * PPR = 4 * 384.5
    private final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

    //Constraints for turret wiring based off of home position (0 ticks)
    private final double MAX_TICKS_RIGHT = 0;
    private final double MAX_TICKS_LEFT = 0;

    private boolean searchingRight = true;

    public TurretSubsystem(HardwareMap hardwareMap, boolean isAutonomous) {
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(POSITION_TOLERANCE_TICKS);

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
            if (pid.atSetPoint()) {
                searchingRight = !searchingRight;
                if (searchingRight) pid.setSetPoint(MAX_TICKS_RIGHT);
                else pid.setSetPoint(MAX_TICKS_LEFT);
            }
            else setPower(pid.calculate(getCurrentTicks()));
        }
        else if (state == TurretState.TARGET) {
            //Lock onto apriltag and use pid to stay targeted
            double targetTicks = getCurrentTicks() + (LatestGoalData.getHeadingOffsetFromGoal() * TICKS_PER_DEGREE);
            pid.setSetPoint(targetTicks);
            setPower(pid.calculate(getCurrentTicks()));
        }
        else if (state == TurretState.HOME) {
            pid.setSetPoint(0); //Home turret
            setPower(pid.calculate(getCurrentTicks()));
            if (pid.atSetPoint()) {
                setState(TurretState.IDLE);
            }
        }
        else {
            setPower(0.0);
        }

        telemetry.addLine("----turret----");
        telemetry.addData("state", state);
        telemetry.addData("encoder ticks", getCurrentTicks());
    }
}
