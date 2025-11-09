package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.ArtifactSource;
import org.stealthrobotics.library.AnglePIDController;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import androidx.annotation.NonNull;

import java.util.ArrayList;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class SpindexerSubsystem extends StealthSubsystem {
    private final DcMotorEx spindexerMotor;

    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.0;

    public static double spindexerSetpoint = 0.0;

    private final double TICKS_PER_REVOLUTION = 537.7; //Gobilda 312 RPM Yellow Jacket
    private final double ANGLE_TOLERANCE_DEGREES = 2;

    private final AnglePIDController pid;

    //TODO: Set Starting Configuration
    private final Slot slot1 = new Slot(Artifact.EMPTY, 0, 180, 1);
    private final Slot slot2 = new Slot(Artifact.EMPTY, 240, 60, 2);
    private final Slot slot3 = new Slot(Artifact.EMPTY, 120, 300, 3);

    //Variables to keep track of which slots are where
    private Slot intakeSlot = null;
    private Slot shooterSlot = null;

    private static class Slot {
        private Artifact artifact;
        private final double intakePosition, shootPosition;
        private final int id;

        public Slot(Artifact artifact, double intakePosition, double shootPosition, int id) {
            this.artifact = artifact;
            this.intakePosition = intakePosition;
            this.shootPosition = shootPosition;
            this.id = id;
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
            return String.valueOf(id);
        }
    }

    public SpindexerSubsystem(HardwareMap hardwareMap, boolean isAutonomous) {
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        spindexerMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        spindexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pid = new AnglePIDController(kP, kI, kD);
        pid.setPositionTolerance(ANGLE_TOLERANCE_DEGREES);

        if (isAutonomous) resetEncoder();
    }

    private void resetEncoder() {
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    //Position in degrees from [-180, 180)
    public double getCurrentPosition() {
        return AngleUnit.normalizeDegrees((spindexerMotor.getCurrentPosition() / TICKS_PER_REVOLUTION) * 360);
    }

    //Rotate the nearest empty slot to the intake
    public Command rotateEmptyToIntake() {
        return this.runOnce(() -> {
            Slot slot = getNearestEmptySlot(ArtifactSource.INTAKE);
            if (slot != null) {
                pid.setSetPoint(slot.getIntakePosition());
                intakeSlot = slot;
            }
        }).andThen(run(() -> setPower(pid.calculate(getCurrentPosition()))).interruptOn(pid::atSetPoint)).andThen(new InstantCommand(() -> setPower(0)));
    }

    //Rotate the nearest empty slot to the shooter
    public Command rotateEmptyToShooter() {
        return this.runOnce(() -> {
            Slot slot = getNearestEmptySlot(ArtifactSource.SHOOTER);
            if (slot != null) {
                pid.setSetPoint(slot.getShootPosition());
                shooterSlot = slot;
            }
        }).andThen(run(() -> setPower(pid.calculate(getCurrentPosition()))).interruptOn(pid::atSetPoint)).andThen(new InstantCommand(() -> setPower(0)));
    }


    // Rotate the nearest artifact of the specified color to the shooter position
    public Command rotateArtifactToShoot(Artifact artifactColor) {
        return this.runOnce(() -> {
            Slot slot = getNearestFilledSlotToShooter(artifactColor);
            if (slot != null) {
                pid.setSetPoint(slot.getShootPosition());
                shooterSlot = slot;
            }
        }).andThen(run(() -> setPower(pid.calculate(getCurrentPosition()))).interruptOn(pid::atSetPoint)).andThen(new InstantCommand(() -> setPower(0)));
    }

    // Rotate the nearest artifact to the shooter position regardless of color
    public Command rotateClosestArtifactToShoot() {
        return this.runOnce(() -> {
            Slot slot = getNearestFilledSlotToShooter();
            if (slot != null) {
                pid.setSetPoint(slot.getShootPosition());
                shooterSlot = slot;
            }
        }).andThen(run(() -> setPower(pid.calculate(getCurrentPosition()))).interruptOn(pid::atSetPoint)).andThen(new InstantCommand(() -> setPower(0)));
    }

    //Return the nearest empty slot to the desired position (intake/shooter)
    private Slot getNearestEmptySlot(ArtifactSource source) {
        ArrayList<Slot> emptySlots = getSlotsWithArtifact(Artifact.EMPTY);

        Slot nearestSlot = null;
        double minDistance = Double.MAX_VALUE;

        for (Slot slot : emptySlots) {
            //Calculate the shortest arc between the slot's (intake/shoot) position and the current spindexer position
            double distance = (source == ArtifactSource.SHOOTER) ?
                    Math.min((slot.getShootPosition() - getCurrentPosition() + 360) % 360, (getCurrentPosition() - slot.getShootPosition() + 360) % 360) :
                    Math.min((slot.getIntakePosition() - getCurrentPosition() + 360) % 360, (getCurrentPosition() - slot.getIntakePosition() + 360) % 360);
            if (distance < minDistance) {
                nearestSlot = slot;
                minDistance = distance;
            }
        }
        return nearestSlot;
    }

    private Slot getNearestFilledSlotToShooter(Artifact artifact) {
        ArrayList<Slot> slots = getSlotsWithArtifact(artifact);

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

    private Slot getNearestFilledSlotToShooter() {
        ArrayList<Slot> slots = getSlotsWithArtifact(Artifact.GREEN);
        slots.addAll(getSlotsWithArtifact(Artifact.PURPLE));

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

    //Return a list of all the slots with the desired artifact type
    private ArrayList<Slot> getSlotsWithArtifact(Artifact color) {
        ArrayList<Slot> slots = new ArrayList<>();
        if (slot1.getArtifact() == color)
            slots.add(slot1);
        if (slot2.getArtifact() == color)
            slots.add(slot2);
        if (slot3.getArtifact() == color)
            slots.add(slot3);

        return slots;
    }

    //Update the spindexer's slot states depending on whether we are intaking/shooting an artifact
    public void updateArtifactState(Artifact artifact, ArtifactSource source) {
        if (source == ArtifactSource.INTAKE) {
            intakeSlot.setArtifact(artifact);
        }
        else {
            shooterSlot.setArtifact(artifact);
        }
    }

    public boolean isFull() {
        return size() == 3;
    }

    public boolean isEmpty() {
        return size() == 0;
    }

    //Returns the number of artifacts in the spindexer
    public int size() {
        int size = 0;
        if (slot1.getArtifact() != Artifact.EMPTY) size++;
        if (slot2.getArtifact() != Artifact.EMPTY) size++;
        if (slot3.getArtifact() != Artifact.EMPTY) size++;
        return size;
    }

    //Returns any unnecessary artifact colors for a motif
    public ArrayList<Artifact> getExtraArtifacts() {
        ArrayList<Artifact> extra = new ArrayList<>();
        int g = 0, p = 0;

        if (slot1.getArtifact() == Artifact.GREEN) g++;
        else if (slot1.getArtifact() == Artifact.PURPLE) p++;
        if (slot2.getArtifact() == Artifact.GREEN) g++;
        else if (slot2.getArtifact() == Artifact.PURPLE) p++;
        if (slot2.getArtifact() == Artifact.GREEN) g++;
        else if (slot2.getArtifact() == Artifact.PURPLE) p++;

        while (g --> 1) extra.add(Artifact.GREEN);
        if (p > 2) extra.add(Artifact.PURPLE);

        return extra;
    }

    //Has all the needed artifact colors to make a motif
    public boolean hasMotifColors() {
        if (slot1.getArtifact() == Artifact.GREEN && slot2.getArtifact() == Artifact.PURPLE && slot3.getArtifact() == Artifact.PURPLE)
            return true;
        if (slot1.getArtifact() == Artifact.PURPLE && slot2.getArtifact() == Artifact.GREEN && slot3.getArtifact() == Artifact.PURPLE)
            return true;
        return slot1.getArtifact() == Artifact.PURPLE && slot2.getArtifact() == Artifact.PURPLE && slot3.getArtifact() == Artifact.GREEN;
    }

    private void setPower(double power) {
        spindexerMotor.setPower(power);
    }

    @Override
    public void periodic() {
        pid.setSetPoint(spindexerSetpoint);
        if (!pid.atSetPoint()) {
            setPower(pid.calculate(getCurrentPosition()));
        }

        telemetry.addLine("----spindexer----");
        telemetry.addData("ticks", spindexerMotor.getCurrentPosition());
        telemetry.addData("angle", getCurrentPosition());
        telemetry.addData("slot 1: ", slot1.getArtifact());
        telemetry.addData("slot 2: ", slot2.getArtifact());
        telemetry.addData("slot 3: ", slot3.getArtifact());
    }
}
