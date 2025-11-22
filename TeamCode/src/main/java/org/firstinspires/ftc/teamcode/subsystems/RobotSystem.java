package org.firstinspires.ftc.teamcode.subsystems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Artifact;
import org.firstinspires.ftc.teamcode.IntakeState;
import org.firstinspires.ftc.teamcode.LEDState;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.ShooterState;
import org.firstinspires.ftc.teamcode.TurretState;
import org.stealthrobotics.library.StealthSubsystem;

import java.util.LinkedList;
import java.util.Queue;
import java.util.function.DoubleSupplier;

public class RobotSystem extends StealthSubsystem {
    private RobotState robotState = RobotState.IDLE;

    public final DriveSubsystem drive;
    public final TurretSubsystem turret;
    public final IntakeSubsystem intake;
    public final ShooterSubsystem shooter;
    public final SpindexerSubsystem spindexer;
    public final VisionSubsystem vision;
    public final LEDSubsystem led;

    //Human controller inputs to alter the state machine (teleop only)
    private final Trigger intakeTrigger;
    private final Trigger outtakeTrigger;
    private final Trigger shootPatternTrigger;
    private final Trigger shootRapidTrigger;

    //State triggers that become true when the robotState becomes their state
    private final Trigger isIDLE = new Trigger(() -> robotState == RobotState.IDLE);
    private final Trigger isINTAKE = new Trigger(() -> robotState == RobotState.INTAKE);
    private final Trigger isOUTTAKE = new Trigger(() -> robotState == RobotState.OUTTAKE);
    private final Trigger isPRE_PATTERN = new Trigger(() -> robotState == RobotState.PRE_PATTERN);
    private final Trigger isPRE_RAPID = new Trigger(() -> robotState == RobotState.PRE_RAPID);
    private final Trigger isSHOOT = new Trigger(() -> robotState == RobotState.SHOOT);

    //Queue that keeps track of the artifacts we want to shoot and what order to shoot them in
    private final Queue<Artifact> shootingQueue = new LinkedList<>();

    //Boolean to keep track of when to process shooting state transitions or not
    private boolean isShooting = false;

    //Time in between shots (essentially the time for the loader to reach its position plus some extra tolerance)
    private final int LOADER_TRAVEL_TIME_MS = 700;

    //It is in fact, not an infinite state machine
    public enum RobotState {
        IDLE,
        INTAKE,
        OUTTAKE,
        PRE_RAPID,
        PRE_PATTERN,
        SHOOT
    }

    public RobotSystem(HardwareMap hardwareMap, Trigger intakeTrigger, Trigger outtakeTrigger, Trigger shootPatternTrigger, Trigger shootRapidTrigger) {
        drive = new DriveSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        spindexer = new SpindexerSubsystem(hardwareMap);
        shooter = new ShooterSubsystem(hardwareMap);
        turret = new TurretSubsystem(hardwareMap);
        vision = new VisionSubsystem(hardwareMap);
        led = new LEDSubsystem(hardwareMap);

        this.intakeTrigger = intakeTrigger;
        this.outtakeTrigger = outtakeTrigger;
        this.shootPatternTrigger = shootPatternTrigger;
        this.shootRapidTrigger = shootRapidTrigger;

        turret.setState(TurretState.TARGET); //Always target throughout the match

        configureStateMachine(intakeTrigger == null);
    }

    public RobotSystem(HardwareMap hardwareMap) {
        this(hardwareMap, null, null, null, null);
    }

    private void configureStateMachine(boolean isAutonomous) {
        if (isAutonomous) configAutonomousStateBehavior();
        else configStateBehavior();
    }

    public void setDriverControl(DoubleSupplier x, DoubleSupplier y, DoubleSupplier rotation) {
        drive.setDefaultCommand(drive.driveTeleop(x, y, rotation));
    }

    public RobotState getState() {
        return robotState;
    }

    private Command setRobotState(RobotState newState) {
        return runOnce(() -> robotState = newState);
    }

    private void configStateBehavior() {
        // IDLE TRANSITIONS
        {
            //Set subsystems to their idle states
            isIDLE
                    .whenActive(intake.setState(IntakeState.IDLE))
                    .whenActive(shooter.setState(ShooterState.IDLE));

            isIDLE
                    .and(intakeTrigger)
                    .and(new Trigger(() -> !spindexer.isFull()))
                    .whenActive(setRobotState(RobotState.INTAKE));

            isIDLE
                    .and(outtakeTrigger)
                    .whenActive(setRobotState(RobotState.OUTTAKE));

            isIDLE
                    .and(shootRapidTrigger)
                    .and(new Trigger(() -> !spindexer.isEmpty()))
                    .whenActive(setRobotState(RobotState.PRE_RAPID));

            isIDLE
                    .and(shootPatternTrigger)
                    .and(new Trigger(() -> !spindexer.isEmpty()))
                    .whenActive(setRobotState(RobotState.PRE_PATTERN));
        }

        // INTAKE STATE LOGIC
        {
            //Exit conditions for intake state
            isINTAKE
                    .and(intakeTrigger.negate()).or(new Trigger(spindexer::isFull))
                    .whenActive(setRobotState(RobotState.IDLE));

            //When we enter intake state, make sure we rotate an empty slot to the intake and start the intake spinning
            isINTAKE
                    .whenActive(spindexer.rotateEmptyToIntake())
                    .whenActive(intake.setState(IntakeState.INTAKE));

            //If we have an empty slot ready, continually check for artifacts
            isINTAKE
                    .and(new Trigger(spindexer::atSetpoint))
                    .and(new Trigger(() -> intake.getSensedArtifact() != Artifact.EMPTY))
                    .whenActive(new InstantCommand(() -> spindexer.intakeArtifact(intake.getSensedArtifact())))
                    .whenActive(new ConditionalCommand(spindexer.rotateEmptyToIntake(), new InstantCommand(), () -> !spindexer.isFull()));
        }

        // OUTTAKE STATE LOGIC
        {
            //Exit conditions for outtake state
            isOUTTAKE
                    .and(outtakeTrigger.negate())
                    .whenActive(setRobotState(RobotState.IDLE));

            isOUTTAKE
                    .whenActive(intake.setState(IntakeState.OUTTAKE));
        }

        // PRE_RAPID STATE LOGIC
        {
            isPRE_RAPID.whenActive(
                    new InstantCommand(() -> shootingQueue.addAll(spindexer.getRapidShootList()))
                    .andThen(setRobotState(RobotState.SHOOT)));
        }

        // PRE_PATTERN STATE LOGIC
        {
            isPRE_PATTERN.whenActive(
                    new InstantCommand(() -> {
                        if (spindexer.hasMotifColors())
                            shootingQueue.addAll(Motif.getPatternList());
                        else
                            shootingQueue.addAll(spindexer.getExtraArtifacts());
                    }).andThen(setRobotState(RobotState.SHOOT))
            );
        }

        // SHOOT STATE LOGIC
        {
            //Exit conditions for shoot state
            isSHOOT
                    .and(new Trigger(() -> spindexer.isEmpty() && !isShooting))
                    .whenActive(setRobotState(RobotState.IDLE));

            isSHOOT
                    .whenActive(spindexer.rotateArtifactToShoot(shootingQueue))
                    .whenActive(shooter.setState(ShooterState.SHOOT));

            isSHOOT
                    .and(new Trigger(() -> spindexer.atSetpoint() && !isShooting))
                    .whenActive(
                            new InstantCommand(() -> isShooting = true)
                                    .andThen(new WaitUntilCommand(shooter::atVelocity).withTimeout(5000)) //Wait for shooter to fully spin up to speed
                                    .andThen(intake.setState(IntakeState.TRANSFER))
                                    .andThen(new WaitCommand(LOADER_TRAVEL_TIME_MS))
                                    .andThen(intake.setState(IntakeState.IDLE))
                                    .andThen(new WaitCommand(LOADER_TRAVEL_TIME_MS))
                                    .andThen(spindexer.shootArtifact())
                                    .andThen(new ConditionalCommand(spindexer.rotateArtifactToShoot(shootingQueue), shooter.setState(ShooterState.IDLE), () -> !spindexer.isEmpty()))
                                    .andThen(new InstantCommand(() -> isShooting = false))
                    );
        }
    }

    //Essentially the same as teleop but with the triggers removed (must manually call to change states in a command group)
    private void configAutonomousStateBehavior() {
        // IDLE TRANSITIONS
        {
            //Set subsystems to their idle states
            isIDLE
                    .whenActive(intake.setState(IntakeState.IDLE))
                    .whenActive(shooter.setState(ShooterState.IDLE));
        }

        // INTAKE STATE LOGIC
        {
            //Exit conditions for intake state
            isINTAKE
                    .and(new Trigger(spindexer::isFull))
                    .whenActive(setRobotState(RobotState.IDLE));

            //When we enter intake state, make sure we rotate an empty slot to the intake and start the intake spinning
            isINTAKE
                    .whenActive(spindexer.rotateEmptyToIntake())
                    .whenActive(intake.setState(IntakeState.INTAKE));

            //If we have an empty slot ready, continually check for artifacts
            isINTAKE
                    .and(new Trigger(spindexer::atSetpoint))
                    .and(new Trigger(() -> intake.getSensedArtifact() != Artifact.EMPTY))
                    .whenActive(new InstantCommand(() -> spindexer.intakeArtifact(intake.getSensedArtifact())))
                    .whenActive(new ConditionalCommand(spindexer.rotateEmptyToIntake(), new InstantCommand(), () -> !spindexer.isFull()));
        }

        // OUTTAKE STATE LOGIC
        {
            isOUTTAKE
                    .whenActive(intake.setState(IntakeState.OUTTAKE));
        }

        // PRE_RAPID STATE LOGIC
        {
            isPRE_RAPID.whenActive(
                    new InstantCommand(() -> shootingQueue.addAll(spindexer.getRapidShootList()))
                            .andThen(setRobotState(RobotState.SHOOT)));
        }

        // PRE_PATTERN STATE LOGIC
        {
            isPRE_PATTERN.whenActive(
                    new InstantCommand(() -> {
                        if (spindexer.hasMotifColors())
                            shootingQueue.addAll(Motif.getPatternList());
                        else
                            shootingQueue.addAll(spindexer.getExtraArtifacts());
                    }).andThen(setRobotState(RobotState.SHOOT))
            );
        }

        // SHOOT STATE LOGIC
        {
            //Exit conditions for shoot state
            isSHOOT
                    .and(new Trigger(() -> spindexer.isEmpty() && !isShooting))
                    .whenActive(setRobotState(RobotState.IDLE));

            isSHOOT
                    .whenActive(spindexer.rotateArtifactToShoot(shootingQueue))
                    .whenActive(shooter.setState(ShooterState.SHOOT));

            isSHOOT
                    .and(new Trigger(() -> spindexer.atSetpoint() && !isShooting))
                    .whenActive(
                            new InstantCommand(() -> isShooting = true)
                                    .andThen(new WaitUntilCommand(shooter::atVelocity)) //Wait for shooter to fully spin up to speed
                                    .andThen(intake.setState(IntakeState.TRANSFER))
                                    .andThen(new WaitCommand(LOADER_TRAVEL_TIME_MS))
                                    .andThen(intake.setState(IntakeState.IDLE))
                                    .andThen(new WaitCommand(LOADER_TRAVEL_TIME_MS))
                                    .andThen(spindexer.shootArtifact())
                                    .andThen(new ConditionalCommand(spindexer.rotateArtifactToShoot(shootingQueue), new InstantCommand(), () -> !spindexer.isEmpty()))
                                    .andThen(new InstantCommand(() -> isShooting = false))
                    );
        }
    }

    private void printTelemetry() {
        telemetry.addLine("<h5>----robot system----</h5>");
        telemetry.addData("<h5>state", robotState + "</h5>");
        telemetry.addData("<h2>shooting", isShooting + "</h2>");
    }

    @Override
    public void periodic() {
        //Led color logic
        if (shooter.atVelocity())
            led.setState(LEDState.GREEN);
        else
            led.setState(LEDState.RED);

        printTelemetry();
    }
}
