package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Artifact;
import org.stealthrobotics.library.StealthSubsystem;
import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;
import static java.lang.Math.abs;

import java.util.ArrayList;

public class SpindexerSubsystem extends StealthSubsystem {
    private final RevColorSensorV3 colorSensor;
    private final CRServo servo1;
    private final CRServo servo2;

    private final double[] DISTANCE_THRESHOLD_INCHES = {0.0, 0.0};

    private int currentIntakeIndex = 0;
    private int currentShootIndex = 0;

    private final ArrayList<Artifact> spindexer = new ArrayList<>();

    private boolean isMoving = false;

    public SpindexerSubsystem(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(RevColorSensorV3.class, "colorSensor");
        servo1 = hardwareMap.get(CRServo.class, "spindexerServo1");
        servo2 = hardwareMap.get(CRServo.class, "spindexerServo2");

        //Initial colors
        spindexer.add(Artifact.EMPTY);
        spindexer.add(Artifact.EMPTY);
        spindexer.add(Artifact.EMPTY);
    }

    public void rotateSlotToIntake(int slotNumber) {
        boolean shortestIsLeft = ((currentIntakeIndex - slotNumber) % 3 < (currentIntakeIndex + slotNumber) % 3);
        int rotations = abs(currentIntakeIndex - slotNumber);

        currentIntakeIndex = slotNumber;

        if (shortestIsLeft)
            for (int i = rotations; i > 0; i--)
                rotateLeft();
        else
            for (int i = rotations; i > 0; i--)
                rotateRight();
    }

    public void loadShooter(Artifact color) {
        if (spindexer.contains(color)) {
            //rotate closest color to the shooter
        }
    }

    private void rotateLeft() {
        new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> servo1.setPower(-1)),
                        new InstantCommand(() -> servo2.setPower(-1))
                ),
                new WaitUntilCommand(this::detectedGamePiece)
        );
    }

    private void rotateRight() {
        new SequentialCommandGroup(
                new ParallelCommandGroup(
                        new InstantCommand(() -> servo1.setPower(1)),
                        new InstantCommand(() -> servo2.setPower(1))
                ),
                new WaitUntilCommand(this::detectedGamePiece)
        );
    }

    //Use color + distance color sensor measurement to accurately detect when a slot is full
    private boolean detectedGamePiece() {
        double distance = colorSensor.getDistance(DistanceUnit.INCH);
        double[] detectedColor = {colorSensor.red(), colorSensor.green(), colorSensor.blue()};

        return (distance > DISTANCE_THRESHOLD_INCHES[0] && distance < DISTANCE_THRESHOLD_INCHES[1]);
    }

    public int getCurrentSlot() {
        return currentIntakeIndex;
    }

    private void intakeArtifact(Artifact artifact) {
        spindexer.set(currentIntakeIndex, artifact);
    }

    private void shootArtifact() {
        spindexer.set(currentShootIndex, Artifact.EMPTY);
    }

    @Override
    public void periodic() {
        telemetry.addData("IndexSlot: ", currentIntakeIndex);
        telemetry.addData("Moving: ", isMoving);
    }
}
