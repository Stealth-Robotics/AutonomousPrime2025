package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Artifact;
import org.stealthrobotics.library.AnglePIDController;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import java.util.ArrayList;

public class SpindexerSubsystem extends StealthSubsystem {
    private final CRServo servo1;
    private final CRServo servo2;

    private final DcMotorEx encoder;

    //PID constants
    public static double kP = 0.0008;
    public static double kI = 0.0;
    public static double kD = 0.0;

    //Variables to keep track of the state of the slots
    private Slot intakeSlot = null;
    private Slot shooterSlot = null;

    private final double SLOT_ANGLE_CONSTANT = 120.0;
    private final double TICKS_PER_REVOLUTION = 8192;

    private final double ANGLE_TOLERANCE = 0.5; //TODO: Tune for accuracy

    private final AnglePIDController pid;

    //TODO: Starting Configuration
    private final Slot slot1 = new Slot(Artifact.EMPTY, 0, 0.0);
    private final Slot slot2 = new Slot(Artifact.EMPTY, 120, 0.0);
    private final Slot slot3 = new Slot(Artifact.EMPTY, -120, 0.0);

    private static class Slot {
        private Artifact artifact;
        private final double intakePosition, shootPosition;

        public Slot(Artifact artifact, double intakePosition, double shootPosition) {
            this.artifact = artifact;
            this.intakePosition = intakePosition;
            this.shootPosition = shootPosition;
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
    }

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(CRServo.class, "spindexerServo1");
        servo2 = hardwareMap.get(CRServo.class, "spindexerServo2");
        encoder = hardwareMap.get(DcMotorEx.class, "spindexerEncoder"); //TODO: Change to correct motor port

        pid = new AnglePIDController(kP, kI, kD);
        pid.setPositionTolerance(ANGLE_TOLERANCE);
    }

    //Position in degrees from [0, 360)
    public double getCurrentPosition() {
        return wrapAngle((encoder.getCurrentPosition() / TICKS_PER_REVOLUTION) * 360);
    }

    private double wrapAngle(double theta) {
        while (theta >= 360) theta -= 360;
        while (theta < 0) theta += 360;
        return theta;
    }

    //Rotate the nearest empty slot to the intake if it exists
    public Command rotateEmptyToIntake() {
        return this.runOnce(() -> {
            Slot slot = nearestEmptySlot();
            if (slot != null) {
                pid.setSetPoint(slot.getIntakePosition());
            }
        }).andThen(run(() -> setPower(pid.calculate(getCurrentPosition()))).interruptOn(pid::atSetPoint));
    }

    // Rotate the nearest artifact of the specified color to the shooter position
    public Command rotateArtifactToShoot(Artifact artifactColor) {
        return this.runOnce(() -> {
            Slot slot = nearestFilledSlot(artifactColor);
            if (slot != null) {
                pid.setSetPoint(slot.getShootPosition());
            }
        }).andThen(run(() -> setPower(pid.calculate(getCurrentPosition()))).interruptOn(pid::atSetPoint));
    }

    //Return the nearest empty slot to the intake position
    private Slot nearestEmptySlot() {
        ArrayList<Slot> emptySlots = new ArrayList<>();
        if (slot1.getArtifact() == Artifact.EMPTY)
            emptySlots.add(slot1);
        if (slot2.getArtifact() == Artifact.EMPTY)
            emptySlots.add(slot2);
        if (slot3.getArtifact() == Artifact.EMPTY)
            emptySlots.add(slot3);

        Slot nearestSlot = null;
        double minDistance = Double.MAX_VALUE;

        for (Slot slot : emptySlots) {
            //Calculate the shortest arc between the slot intake position and the current position
            double distance = Math.min((slot.getIntakePosition() - getCurrentPosition() + 360) % 360, (getCurrentPosition() - slot.getIntakePosition() + 360) % 360);
            if (distance < minDistance) {
                nearestSlot = slot;
                minDistance = distance;
            }
        }
        return nearestSlot;
    }

    private Slot nearestFilledSlot(Artifact artifact) {
        ArrayList<Slot> slots = new ArrayList<>();
        if (slot1.getArtifact() == artifact)
            slots.add(slot1);
        if (slot2.getArtifact() == artifact)
            slots.add(slot2);
        if (slot3.getArtifact() == artifact)
            slots.add(slot3);

        Slot nearestSlot = null;
        double minDistance = Double.MAX_VALUE;

        for (Slot slot : slots) {
            //Calculate the shortest arc between the slot shoot position and the current position
            double distance = Math.min((slot.getShootPosition() - getCurrentPosition() + 360) % 360, (getCurrentPosition() - slot.getShootPosition() + 360) % 360);
            if (distance < minDistance) {
                nearestSlot = slot;
                minDistance = distance;
            }
        }
        return nearestSlot;
    }

    //Set the power of both servos in parallel
    private void setPower(double power) {
        servo1.setPower(power);
        servo2.setPower(power);
    }

    @Override
    public void periodic() {
        telemetry.addData("position: ", getCurrentPosition());
        telemetry.addData("slot 1: ", slot1.getArtifact());
        telemetry.addData("slot 2: ", slot2.getArtifact());
        telemetry.addData("slot 3: ", slot3.getArtifact());
        telemetry.addData("atSetpoint: ", pid.atSetPoint());

        // ! For graphing
//        FtcDashboard dashboard = FtcDashboard.getInstance();
//        Telemetry dashboardTelemetry = dashboard.getTelemetry();
//
//        dashboardTelemetry.addData("target", pid.getSetPoint());
//        dashboardTelemetry.addData("current", getCurrentPosition());
//        dashboardTelemetry.update();
    }
}
