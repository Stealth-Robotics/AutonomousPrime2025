package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
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

import java.util.ArrayList;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class SpindexerSubsystem extends StealthSubsystem {
    private final CRServo servo1;
    private final CRServo servo2;

    private final DcMotorEx encoder;

    //PID constants
    public static double kP = 0.0034;
    public static double kI = 0.0001;
    public static double kD = 0.00025;
    public static double kF = 0.051;

    // ! Only for testing
    public static double testPosition = 0;

    //Variables to keep track of the state of the slots
    private Slot intakeSlot = null;
    private Slot shooterSlot = null;

    private final double TICKS_PER_REVOLUTION = 8192;

    private final double ANGLE_TOLERANCE = 2;

    private final AnglePIDController pid;

    //TODO: Starting Configuration
    private final Slot slot1 = new Slot(Artifact.EMPTY, 0, 180);
    private final Slot slot2 = new Slot(Artifact.EMPTY, 120, 60);
    private final Slot slot3 = new Slot(Artifact.EMPTY, 240, 300);

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

        pid = new AnglePIDController(kP, kI, kD, kF);
        pid.setPositionTolerance(ANGLE_TOLERANCE);

        resetEncoder();
    }

    private void resetEncoder() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //Position in degrees from [-180, 180)
    public double getCurrentPosition() {
        return AngleUnit.normalizeDegrees((encoder.getCurrentPosition() / TICKS_PER_REVOLUTION) * 360);
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

    public Command rotateClosestArtifactToShoot() {
        return this.runOnce(() -> {
            Slot slot = nearestFilledSlot();
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

    private Slot nearestFilledSlot() {
        ArrayList<Slot> slots = new ArrayList<>();
        if (slot1.getArtifact() != Artifact.EMPTY)
            slots.add(slot1);
        if (slot2.getArtifact() != Artifact.EMPTY)
            slots.add(slot2);
        if (slot3.getArtifact() != Artifact.EMPTY)
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

    public Command updateSlotState(Artifact artifact, boolean intake, boolean outtake) {
        return this.runOnce(() -> {
            if (intake)
                intakeSlot.setArtifact(artifact);
            else if (outtake)
                shooterSlot.setArtifact(artifact);
        });
    }

    public boolean isFull() {
        return gamepieceCount() == 3;
    }

    public boolean isEmpty() {
        return gamepieceCount() == 0;
    }

    public int gamepieceCount() {
        int size = 0;
        if (slot1.getArtifact() != Artifact.EMPTY) size++;
        if (slot2.getArtifact() != Artifact.EMPTY) size++;
        if (slot3.getArtifact() != Artifact.EMPTY) size++;
        return size;
    }

    public boolean hasMotifColors() {
        if (slot1.getArtifact() == Artifact.GREEN && slot2.getArtifact() == Artifact.PURPLE && slot3.getArtifact() == Artifact.PURPLE)
            return true;
        if (slot1.getArtifact() == Artifact.PURPLE && slot2.getArtifact() == Artifact.GREEN && slot3.getArtifact() == Artifact.PURPLE)
            return true;
        return slot1.getArtifact() == Artifact.PURPLE && slot2.getArtifact() == Artifact.PURPLE && slot3.getArtifact() == Artifact.GREEN;
    }

    //Set the power of both servos in parallel
    private void setPower(double power) {
        servo1.setPower(-power);
        servo2.setPower(-power);
    }

    @Override
    public void periodic() {
        telemetry.addData("position: ", getCurrentPosition());
        telemetry.addData("slot 1: ", slot1.getArtifact());
        telemetry.addData("slot 2: ", slot2.getArtifact());
        telemetry.addData("slot 3: ", slot3.getArtifact());
        telemetry.addData("atPosition: ", pid.atSetPoint());
        telemetry.addData("error", pid.getError());

        pid.setSetPoint(testPosition);
        setPower(pid.calculate(getCurrentPosition()));

        // ! For graphing
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("target", pid.getSetPoint());
        dashboardTelemetry.addData("current", getCurrentPosition());
        dashboardTelemetry.update();
    }
}
