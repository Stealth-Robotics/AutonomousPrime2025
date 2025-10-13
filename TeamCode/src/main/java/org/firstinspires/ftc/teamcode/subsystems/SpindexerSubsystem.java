package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.math.MathFunctions;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.stealthrobotics.library.AnglePIDController;
import org.stealthrobotics.library.Commands;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;
import static java.lang.Math.abs;

import java.util.ArrayList;

@Config
public class SpindexerSubsystem extends StealthSubsystem {
    private final CRServo servo1;
    private final CRServo servo2;
    private final AnalogInput encoder;

    //PID constants
    public static double kP = 0.0008;
    public static double kI = 0.0;
    public static double kD = 0.0;

    //Variables to keep track of the state of the slots
    private Slot slotAtIntake = null;
    private Slot slotAtShooter = null;

    private final double SHOOT_POSITION = 180.0;
    private final double INTAKE_POSITION = 0.0;

    private final double SLOT_ANGLE_CONSTANT = 120.0; //Angle in between all of the slots

    private final double ANGLE_TOLERANCE = 1.0; //TODO: Tune for accuracy
    private final double VELOCITY_TOLERANCE = 5.0; //TODO: Tune for accuracy

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
        encoder = hardwareMap.get(AnalogInput.class, "absoluteEncoder");

        pid = new AnglePIDController(kP, kI, kD);

        pid.setPositionTolerance(ANGLE_TOLERANCE);
        pid.setVelocityTolerance(VELOCITY_TOLERANCE);

        pid.setSetPoint(90);
    }

    //Position in degrees from [0, 360]
    public double getCurrentPosition() {
        return (encoder.getVoltage() / 3.3 * 360.0);
    }

    //Rotate the nearest empty slot to the intake to be filled
    public void rotateEmptyToIntake() {
        Slot slot = nearestEmpty();
        if (slot != null) {
            pid.setSetPoint(slot.getIntakePosition());
            slotAtIntake = slot;
        }
    }

    //Change the artifact state of the slot at the intake position
    public void loadSlot(Artifact artifact) {
        if (slotAtIntake != null) {
            slotAtIntake.setArtifact(artifact);
            slotAtIntake = null;
        }
    }

    //Return the nearest empty slot to the intake position
    private Slot nearestEmpty() {
//        ArrayList<Slot> emptySlots = new ArrayList<>();
//        if (slot1.getArtifact() == Artifact.EMPTY)
//            emptySlots.add(slot1);
//        if (slot2.getArtifact() == Artifact.EMPTY)
//            emptySlots.add(slot2);
//        if (slot3.getArtifact() == Artifact.EMPTY)
//            emptySlots.add(slot3);
//
//        Slot nearestSlot = null;
//        double minDistance = Double.MAX_VALUE;
//
//        for (Slot slot : emptySlots) {
//            double distance = //Find distance from slot intake position to current position
//            if (distance < minDistance) {
//                nearestSlot = slot;
//                minDistance = distance;
//            }
//        }
//
//        return nearestSlot;
        return slot1;
    }

    //Set the power of both servos in parallel
    private void setPower(double power) {
        servo1.setPower(power);
        servo2.setPower(power);
    }

    @Override
    public void periodic() {
        if (!pid.atSetPoint()) {
            setPower(pid.calculate(getCurrentPosition()));
        }

        telemetry.addData("position: ", getCurrentPosition());
        telemetry.addData("slot 1: ", slot1.getArtifact());
        telemetry.addData("slot 2: ", slot2.getArtifact());
        telemetry.addData("slot 3: ", slot3.getArtifact());
        telemetry.addData("atSetpoint: ", pid.atSetPoint());

        // ! For graphing
        FtcDashboard dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("target", pid.getSetPoint());
        dashboardTelemetry.addData("current", getCurrentPosition());
        dashboardTelemetry.update();
    }
}
