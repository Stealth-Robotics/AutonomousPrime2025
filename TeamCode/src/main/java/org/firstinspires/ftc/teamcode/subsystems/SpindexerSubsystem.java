package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.stealthrobotics.library.AnglePIDController;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;
import static java.lang.Math.abs;

import java.util.ArrayList;

public class SpindexerSubsystem extends StealthSubsystem {
    private final CRServo servo1;
    private final CRServo servo2;
    private final AnalogInput encoder;

    //TODO: TUNE
    private final int SHOOT_POSITION = 0; //when spindexer is aligned at setpoint 0, the shooter offset
    private final int INTAKE_POSITION = 0;

    private final AnglePIDController pid;

    private final double ANGLE_TOLERANCE = 0.0;

    //TODO: Starting Configuration
    //TODO: TUNE
    private Slot slot1 = new Slot(Artifact.EMPTY, 0.0, 0.0);
    private Slot slot2 = new Slot(Artifact.EMPTY, 0.0, 0.0);
    private Slot slot3 = new Slot(Artifact.EMPTY, 0.0, 0.0);

    private class Slot {
        private Artifact artifact;
        private final double intakeOffset, shooterOffset;

        public Slot(Artifact artifact, double intakeOffset, double shooterOffset) {
            this.artifact = artifact;
            this.intakeOffset = intakeOffset;
            this.shooterOffset = shooterOffset;
        }

        public void setArtifact(Artifact artifact) {
            this.artifact = artifact;
        }

        public Artifact getArtifact() {
            return artifact;
        }

        public double getIntakeOffset() {
            return intakeOffset;
        }

        public double getShooterOffset() {
            return shooterOffset;
        }
    }

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        servo1 = hardwareMap.get(CRServo.class, "spindexerServo1");
        servo2 = hardwareMap.get(CRServo.class, "spindexerServo2");
        encoder = hardwareMap.get(AnalogInput.class, "absoluteEncoder");

        pid = new AnglePIDController(0.0, 0.0, 0.0);
        pid.setTolerance(ANGLE_TOLERANCE);
    }

    //Get position in degrees
    public double getSpindexerPosition() {
        return (encoder.getVoltage() / 3.3 * 360.0);
    }

    //Rotate the nearest empty slot to the intake to be filled
//    public Command rotateEmptyToIntake() {
//        return new ConditionalCommand(
//                new SequentialCommandGroup(
//
//                ),
//                new InstantCommand(),
//                this::hasEmptySlot
//        );
//    }


    //Return the nearest empty slot
    private Slot nearestEmpty() {
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
            double distanceError = 0.0; // ! Calculate angle error between slot and intake position
            if (distanceError < minDistance) {
                nearestSlot = slot;
                minDistance = distanceError;
            }
        }

        return nearestSlot;
    }

    private Command setPower(double power) {
        return new ParallelCommandGroup(
                new InstantCommand(() -> servo1.setPower(power)),
                new InstantCommand(() -> servo2.setPower(power))
        );
    }

    @Override
    public void periodic() {
        telemetry.addData("position: ", getSpindexerPosition());
    }
}
