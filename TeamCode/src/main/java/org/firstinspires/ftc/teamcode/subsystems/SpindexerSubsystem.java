package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.stealthrobotics.library.AnglePIDController;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import androidx.annotation.NonNull;

import java.util.ArrayList;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class SpindexerSubsystem extends StealthSubsystem {
    private final CRServo servo1;
    private final CRServo servo2;

    private final DcMotorEx encoder;

    //PID constants
    public static double kP = 0.0042;
    public static double kI = 0.0;
    public static double kD = 0.00033;
    public static double kF = 0.0;

    private final double TICKS_PER_REVOLUTION = 8192;

    private final double ANGLE_TOLERANCE = 10;

    private final AnglePIDController pid;

    private boolean intakeMode = true;

    //TODO: Starting Configuration
    private final Slot slot1 = new Slot(Artifact.EMPTY, 0, 180, "slot1");
    private final Slot slot2 = new Slot(Artifact.EMPTY, 240, 60, "slot2");
    private final Slot slot3 = new Slot(Artifact.EMPTY, 120, 300, "slot3");

    private static class Slot {
        private Artifact artifact;
        private final double intakePosition, shootPosition;
        private final String name;

        public Slot(Artifact artifact, double intakePosition, double shootPosition, String name) {
            this.artifact = artifact;
            this.intakePosition = intakePosition;
            this.shootPosition = shootPosition;
            this.name = name;
        }

        public void setArtifact(Artifact artifact) {
            this.artifact = artifact;
        }

        public Artifact getArtifact() {
            return artifact;
        }

        public double getIntakePosition() {
            return intakePosition;
        }

        public double getShootPosition() {
            return shootPosition;
        }

        @NonNull
        @Override
        public String toString() {
            return name;
        }
    }

    public enum SpindexerSlot {
        ONE,
        TWO,
        THREE
    }

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(CRServo.class, "spindexerServo1");
        servo2 = hardwareMap.get(CRServo.class, "spindexerServo2");
        encoder = hardwareMap.get(DcMotorEx.class, "spindexerEncoder"); //TODO: Change to correct motor port

        pid = new AnglePIDController(kP, kI, kD, kF);
        pid.setPositionTolerance(ANGLE_TOLERANCE);

        resetEncoder();
    }

    public Command toggleMode() {
        return this.runOnce(() -> intakeMode = !intakeMode);
    }

    public Command controlSlot(SpindexerSlot slotNum) {
        return new ConditionalCommand(rotateToIntake(slotNum), rotateToShoot(slotNum), () -> intakeMode);
    }

    private void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Position in degrees from [-180, 180)
    public double getCurrentPosition() {
        return AngleUnit.normalizeDegrees((encoder.getCurrentPosition() / TICKS_PER_REVOLUTION) * 360);
    }

    public Command rotateToShoot(SpindexerSlot slotNum) {
        return this.runOnce(() -> {
            Slot slot;
            if (slotNum == SpindexerSlot.ONE) slot = slot1;
            else if (slotNum == SpindexerSlot.TWO) slot = slot2;
            else slot = slot3;
            pid.setSetPoint(slot.getShootPosition());
        }).andThen(run(() -> setPower(pid.calculate(getCurrentPosition()))).interruptOn(pid::atSetPoint)).andThen(new InstantCommand(() -> setPower(0)));
    }

    public Command rotateToIntake(SpindexerSlot slotNum) {
        return this.runOnce(() -> {
            Slot slot;
            if (slotNum == SpindexerSlot.ONE) slot = slot1;
            else if (slotNum == SpindexerSlot.TWO) slot = slot2;
            else slot = slot3;
            pid.setSetPoint(slot.getIntakePosition());
        }).andThen(run(() -> setPower(pid.calculate(getCurrentPosition()))).interruptOn(pid::atSetPoint)).andThen(new InstantCommand(() -> setPower(0)));
    }

    //Set the power of both servos in parallel
    private void setPower(double power) {
        servo1.setPower(-power);
        servo2.setPower(-power);
    }

    @Override
    public void periodic() {
        telemetry.addData("spindexer mode", (intakeMode) ? "intaking" : "shooting");
    }
}
