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
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import androidx.annotation.NonNull;

import java.util.ArrayList;
import java.util.Queue;

@Config
@SuppressWarnings("FieldCanBeLocal")
public class SpindexerSubsystem extends StealthSubsystem {
    private final DcMotorEx spindexerMotor;
    private final PIDController pid;

    public static double kP = 0.00025;
    public static double kI = 0.05;
    public static double kD = 0.000005;
    public static double kS = 0.01;

    private double encoderOffset = 0.0;

    private final double TICKS_PER_REVOLUTION = 8192; //REV Thru Bore
    private final double TICKS_PER_DEGREE = TICKS_PER_REVOLUTION / 360.0;

    private final double POSITION_TOLERANCE_TICKS = 60; //Around 2.3 degrees

    /* Slot numbers increase going counter-clockwise
                    3 ————— 2
                     \     /
                        1
     */

    // Slot containers which track proper intaking/shooting angles and what type of artifacts are where
    private final Slot slot1 = new Slot(Artifact.EMPTY, 0, 180, 1);
    private final Slot slot2 = new Slot(Artifact.EMPTY, -120, 60, 2);
    private final Slot slot3 = new Slot(Artifact.EMPTY, 120, -60, 3);

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

    public void moveSpindexerManually(double amount) {
        encoderOffset += amount;
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
        });
    }

    //Rotate the nearest empty slot to the intake
    public Command rotateEmptyToIntake() {
        return runOnce(() -> {
            Slot slot = getNearestEmptySlot();

            if (slot != null) {
                pid.setSetPoint(slot.getIntakePosition() * TICKS_PER_DEGREE);
                intakeSlot = slot;
            }
        });
    }

    // Rotate the nearest artifact of the specified color to the shooter position
    public Command rotateArtifactToShoot(Queue<Artifact> shootingQueue) {
        return runOnce(() -> {
            Slot slot = getNearestFilledSlotToShooter(shootingQueue.remove());

            if (slot != null) {
                pid.setSetPoint(slot.getShootPosition() * TICKS_PER_DEGREE);
                shooterSlot = slot;
            }
        });
    }

    //Return the nearest empty slot to the intake
    private Slot getNearestEmptySlot() {
        ArrayList<Slot> emptySlots = getSlotsWithArtifact(Artifact.EMPTY);

        Slot nearestSlot = null;
        double minDistance = Double.MAX_VALUE;

        for (Slot slot : emptySlots) {
            //Calculate the shortest arc between the slot's intake position and the current spindexer position
            double distance = Math.abs(((slot.getIntakePosition() - getAngleDegrees() + 540) % 360) - 180);
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
            double distance = Math.abs(((slot.getShootPosition() - getAngleDegrees() + 540) % 360) - 180);
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

        double prevPos = getAngleDegrees();

        for (int i = 0; i < size(); i++) {
            Slot nearestSlot = null;
            double minDistance = Double.MAX_VALUE;

            for (Slot slot : filledSlots) {
                //Calculate the shortest arc between the slot shoot position and the current position
                double distance = Math.abs(((slot.getShootPosition() - prevPos + 540) % 360) - 180);
                if (distance < minDistance) {
                    nearestSlot = slot;
                    minDistance = distance;
                }
            }

            assert nearestSlot != null;

            shootList.add(nearestSlot.getArtifact());
            filledSlots.remove(nearestSlot);

            prevPos = nearestSlot.getShootPosition();
        }

        return shootList;
    }

    //Return a list of all the slots with the desired artifact type
    private ArrayList<Slot> getSlotsWithArtifact(Artifact color) {
        ArrayList<Slot> slots = new ArrayList<>();
        if (slot1.getArtifact().equals(color))
            slots.add(slot1);
        if (slot2.getArtifact().equals(color))
            slots.add(slot2);
        if (slot3.getArtifact().equals(color))
            slots.add(slot3);

        return slots;
    }

    public void intakeArtifact(Artifact artifact) {
       intakeSlot.setArtifact(artifact);
    }

    public Command shootArtifact() {
        return new InstantCommand(() -> shooterSlot.setArtifact(Artifact.EMPTY));
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

    //Has all the needed artifact colors to make a motif
    public boolean hasMotifColors() {
        if (slot1.getArtifact() == Artifact.GREEN && slot2.getArtifact() == Artifact.PURPLE && slot3.getArtifact() == Artifact.PURPLE)
            return true;
        if (slot1.getArtifact() == Artifact.PURPLE && slot2.getArtifact() == Artifact.GREEN && slot3.getArtifact() == Artifact.PURPLE)
            return true;
        return slot1.getArtifact() == Artifact.PURPLE && slot2.getArtifact() == Artifact.PURPLE && slot3.getArtifact() == Artifact.GREEN;
    }

    public boolean atSetpoint() {
        return pid.atSetPoint();
    }

    private void setPower(double power) {
        spindexerMotor.setPower(power);
    }

    @Override
    public void periodic() {
        double pidOutput = pid.calculate(getCurrentTicks());
        double kSFeedforward = kS * Math.signum(pid.getPositionError());

        setPower(pidOutput + kSFeedforward);

        telemetry.addLine("----spindexer----");
        telemetry.addData("error", pid.getPositionError());
        telemetry.addData("setpoint reached", atSetpoint());
        telemetry.addData("current draw (amps)", spindexerMotor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("1", slot1.getArtifact());
        telemetry.addData("2", slot2.getArtifact());
        telemetry.addData("3", slot3.getArtifact());
    }
}
