package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.ArtifactSource;
import org.firstinspires.ftc.teamcode.IntakeState;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import androidx.annotation.NonNull;

import java.util.ArrayList;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class SpindexerSubsystem extends StealthSubsystem {
    private final DcMotorEx spindexerMotor;
    private final PIDController pid;

    public static double kP = 0.0035;
    public static double kI = 0.12;
    public static double kD = 0.0003;

    private double encoderOffset = 0.0;

    private final double TICKS_PER_REVOLUTION = 537.7; //Gobilda 312 RPM Yellow Jacket
    private final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;
    private final double POSITION_TOLERANCE_TICKS = 10;

    /* Slot numbers increase going counter-clockwise
                    3-------2
                     \     /
                        1
     */

    //Autonomous preset artifact locations
    private final Slot slot1 = new Slot(Artifact.GREEN, 0, 180, 1);
    private final Slot slot2 = new Slot(Artifact.PURPLE, -120, 60, 2);
    private final Slot slot3 = new Slot(Artifact.PURPLE, 120, -60, 3);

    //Variables to keep track of which slots are where
    private Slot intakeSlot = slot1;
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

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        spindexerMotor = hardwareMap.get(DcMotorEx.class, "spindexerMotor");
        spindexerMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        pid = new PIDController(kP, kI, kD);
        pid.setTolerance(POSITION_TOLERANCE_TICKS);

        resetEncoder();
    }

    private void resetEncoder() {
        spindexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setArtifactsInSpindexerManually(Artifact slot1, Artifact slot2, Artifact slot3) {
        this.slot1.setArtifact(slot1);
        this.slot2.setArtifact(slot2);
        this.slot3.setArtifact(slot3);
    }

    /** @return the spindexer's artifacts in the order s1, s2, s3 **/
    public Artifact[] getCurrentArtifacts() {
        return new Artifact[] {slot1.getArtifact(), slot2.getArtifact(), slot3.getArtifact()};
    }

    //Essentially re-zeros the motor encoder without homing
    public void setEncoderOffset(double encoderOffset) {
        this.encoderOffset = encoderOffset;
    }

    public double getTicks() {
        return spindexerMotor.getCurrentPosition();
    }

    public double getCurrentTicks() {
        return spindexerMotor.getCurrentPosition() + encoderOffset;
    }

    public double getAngleDegrees() {
        return AngleUnit.normalizeDegrees((getCurrentTicks() / TICKS_PER_REVOLUTION) * 360);
    }

    public Command rotateToSlotNumber(int slotNumber) {
        return this.runOnce(() -> {
            Slot slot = slot3;
            if (slotNumber == 1) slot = slot1;
            if (slotNumber == 2) slot = slot2;

            pid.setSetPoint(slot.getIntakePosition() * TICKS_PER_DEGREE);
            intakeSlot = slot;
        }).andThen(new WaitUntilCommand(this::atPosition).withTimeout(1000));
    }

    //Rotate the nearest empty slot to the intake
    public Command rotateEmptyToIntake() {
        return this.runOnce(() -> {
            Slot slot = getNearestEmptySlot(ArtifactSource.INTAKE);

            assert slot != null;

            pid.setSetPoint(slot.getIntakePosition() * TICKS_PER_DEGREE);
            intakeSlot = slot;
        });
    }

    // Rotate the nearest artifact of the specified color to the shooter position
    public Command rotateArtifactToShoot(Artifact artifactColor) {
        return this.runOnce(() -> {
            Slot slot = getNearestFilledSlotToShooter(artifactColor);

            assert slot != null;

            pid.setSetPoint(slot.getShootPosition() * TICKS_PER_DEGREE);
            shooterSlot = slot;
        });
    }

    // Rotate the nearest artifact to the shooter position regardless of color
    public Command rotateClosestArtifactToShoot() {
        return this.runOnce(() -> {
            Slot slot = getNearestFilledSlotToShooter();

            assert slot != null;

            pid.setSetPoint(slot.getShootPosition() * TICKS_PER_DEGREE);
            shooterSlot = slot;
        });
    }

    //Return the nearest empty slot to the desired position (intake/shooter)
    private Slot getNearestEmptySlot(ArtifactSource source) {
        ArrayList<Slot> emptySlots = getSlotsWithArtifact(Artifact.EMPTY);

        Slot nearestSlot = null;
        double minDistance = Double.MAX_VALUE;

        for (Slot slot : emptySlots) {
            //Calculate the shortest arc between the slot's (intake/shoot) position and the current spindexer position
            double distance = (source == ArtifactSource.SHOOTER) ?
                    Math.min((slot.getShootPosition() - getAngleDegrees() + 360) % 360, (getAngleDegrees() - slot.getShootPosition() + 360) % 360) :
                    Math.min((slot.getIntakePosition() - getAngleDegrees() + 360) % 360, (getAngleDegrees() - slot.getIntakePosition() + 360) % 360);
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
            double distance = Math.min((slot.getShootPosition() - getAngleDegrees() + 360) % 360, (getAngleDegrees() - slot.getShootPosition() + 360) % 360);
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
            double distance = Math.min((slot.getShootPosition() - getAngleDegrees() + 360) % 360, (getAngleDegrees() - slot.getShootPosition() + 360) % 360);
            if (distance < minDistance) {
                nearestSlot = slot;
                minDistance = distance;
            }
        }
        return nearestSlot;
    }

    /** @return A list of the fastest combination to shoot out all of the robot's artifacts **/
    public ArrayList<Artifact> getRapidShootList() {
        ArrayList<Artifact> shootList = new ArrayList<>();

        ArrayList<Slot> filledSlots = getSlotsWithArtifact(Artifact.PURPLE);
        filledSlots.addAll(getSlotsWithArtifact(Artifact.GREEN));

        for (int i = 0; i < size(); i++) {
            Slot nearestSlot = null;
            double minDistance = Double.MAX_VALUE;

            for (Slot slot : filledSlots) {
                //Calculate the shortest arc between the slot shoot position and the current position
                double distance = Math.min((slot.getShootPosition() - getAngleDegrees() + 360) % 360, (getAngleDegrees() - slot.getShootPosition() + 360) % 360);
                if (distance < minDistance) {
                    nearestSlot = slot;
                    minDistance = distance;
                }
            }

            assert nearestSlot != null;
            shootList.add(nearestSlot.getArtifact());
            filledSlots.remove(nearestSlot);
        }

        return shootList;
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

    public Command intakeArtifact(Artifact artifact) {
        return this.runOnce(() -> {
            intakeSlot.setArtifact(artifact);
        });
    }

    public Command shootArtifact() {
        return this.runOnce(() -> {
            shooterSlot.setArtifact(Artifact.EMPTY);
        });
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

    public boolean atPosition() {
        return pid.atSetPoint();
    }

    private void setPower(double power) {
        spindexerMotor.setPower(MathFunctions.clamp(power, -0.4, 0.4));
    }

    @Override
    public void periodic() {
        setPower(pid.calculate(getCurrentTicks()));

        telemetry.addLine("----spindexer----");
        telemetry.addData("ticks", getCurrentTicks());
        telemetry.addData("atPosition", atPosition());
        telemetry.addData("setpoint", pid.getSetPoint());
        telemetry.addData("error", pid.getPositionError());
        telemetry.addData("1", slot1.getArtifact());
        telemetry.addData("2", slot2.getArtifact());
        telemetry.addData("3", slot3.getArtifact());
    }
}
