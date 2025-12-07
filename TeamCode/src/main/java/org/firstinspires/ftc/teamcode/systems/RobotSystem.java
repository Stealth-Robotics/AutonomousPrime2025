package org.firstinspires.ftc.teamcode.systems;

import static org.stealthrobotics.library.opmodes.StealthOpMode.telemetry;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RepeatCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.AutoToTeleopData;
import org.firstinspires.ftc.teamcode.Motif;
import org.firstinspires.ftc.teamcode.PoseEstimator;
import org.firstinspires.ftc.teamcode.enums.Artifact;
import org.firstinspires.ftc.teamcode.enums.IntakeState;
import org.firstinspires.ftc.teamcode.PatternMode;
import org.firstinspires.ftc.teamcode.enums.LEDState;
import org.firstinspires.ftc.teamcode.enums.ShooterState;
import org.firstinspires.ftc.teamcode.enums.TurretState;
import org.stealthrobotics.library.Alliance;
import org.stealthrobotics.library.StealthSubsystem;
import org.stealthrobotics.library.math.filter.Debouncer;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;
import java.util.function.DoubleSupplier;

/**
 * A class that neatly initializes all of the robot's subsystems in one place and allows different subsystem's
 * behavior to be combined in it's periodic. This system can be used in autonomous or tele-op, but for tele-op the state machine
 * transition must not include gamepad triggers.
 *
 * @author Marco Videla
 * @author Ian Nachreiner
 */
public class RobotSystem extends StealthSubsystem {
    private RobotState robotState = RobotState.IDLE;

    private final Debouncer tagDebouncer = new Debouncer(2, Debouncer.DebounceType.kRising);

    //List for storing persistent telemetry to write to the Driver Station in periodic
    private final ArrayList<String> persistentTelemetry = new ArrayList<>();

    //All of the robot's subsystems
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

    //Operator dictates where we are in the motif (0 = first, 1 = middle, 2 = last)
    private int patternIndexOffset = 0;

    //Keep track of whether we have intaked a new artifact (for rumble)
    public boolean justIntaked = false;

    //Time in between shots (essentially the time for the loader to reach its position plus some extra tolerance)
    private final int LOADER_TRAVEL_TIME_MS = 400;

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
        boolean isAuto = (intakeTrigger == null);

        if (!isAuto)
            drive = new DriveSubsystem(hardwareMap);
        else
            drive = null;

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

        configureStateMachine(isAuto);

        turret.switchToOdometryControl();

        //Transfer necessary subsystem data from auto into teleop
        if (!isAuto) {
            spindexer.setArtifactsInSpindexerManually(Artifact.EMPTY, Artifact.EMPTY, Artifact.EMPTY);
            drive.setPose(AutoToTeleopData.endOfAutoPose);
            drive.setAllianceSpecificHeading(Alliance.get());
        }
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

    public Command setPatternOffset(int newOffset) {
        return new InstantCommand(() -> patternIndexOffset = newOffset);
    }

    public RobotState getState() {
        return robotState;
    }

    public Command setRobotState(RobotState newState) {
        return new InstantCommand(() -> robotState = newState);
    }

    private void configStateBehavior() {
        // IDLE TRANSITIONS
        {
            //Set subsystems to their idle states
            isIDLE
                    .whenActive(intake.setState(IntakeState.IDLE))
                    .whenActive(
                            new ConditionalCommand(
                                    shooter.setState(ShooterState.SHOOT),
                                    shooter.setState(ShooterState.IDLE),
                                    () -> spindexer.isFull())
                    );

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
                    .and(new Trigger(spindexer::atIntakeSetpoint))
                    .and(new Trigger(() -> intake.getSensedArtifact() != Artifact.EMPTY))
                    .whenActive(new InstantCommand(() -> justIntaked = true))
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
                    new InstantCommand(() -> {
                        shootingQueue.clear(); //Clear just in case
                        shootingQueue.addAll(spindexer.getRapidShootList());
                    }).andThen(new ConditionalCommand(setRobotState(RobotState.SHOOT), setRobotState(RobotState.IDLE), () -> !shootingQueue.isEmpty()))
            );
        }

        // PRE_PATTERN STATE LOGIC
        {
            isPRE_PATTERN.whenActive(
                    new InstantCommand(() -> {
                        shootingQueue.clear(); //Clear just in case
                        shootingQueue.addAll(PatternMode.getPatternSequence(patternIndexOffset, spindexer.greenCount(), spindexer.purpleCount()));
                    }).andThen(new ConditionalCommand(setRobotState(RobotState.SHOOT), setRobotState(RobotState.IDLE), () -> !shootingQueue.isEmpty()))
            );
        }

        // SHOOT STATE LOGIC
        {
            isSHOOT
                    .and(new Trigger(() -> spindexer.isEmpty() || shootingQueue.isEmpty()))
                    .whenActive(shooter.setState(ShooterState.IDLE))
                    .whenActive(intake.setState(IntakeState.IDLE))
                    .whenActive(setRobotState(RobotState.IDLE));

            isSHOOT
                    .whenActive(spindexer.rotateArtifactToShoot(shootingQueue))
                    .whenActive(shooter.setState(ShooterState.SHOOT));

            isSHOOT
                    .and(new Trigger(() -> shooter.isReadyToShoot()))
                    .whenActive(
                            new SequentialCommandGroup(
                                    new WaitUntilCommand(shooter::atVelocity).withTimeout(4000),
                                    new RepeatCommand(
                                            new SequentialCommandGroup(
                                                    new WaitUntilCommand(spindexer::atSetpoint).withTimeout(250),
                                                    new WaitUntilCommand(shooter::atVelocity).withTimeout(500),
                                                    new WaitCommand(300),
                                                    intake.setState(IntakeState.TRANSFER),
                                                    new WaitCommand(LOADER_TRAVEL_TIME_MS),
                                                    intake.setState(IntakeState.OUTTAKE),
                                                    new WaitCommand(200),
                                                    new InstantCommand(() -> {
                                                        spindexer.shootArtifact();
                                                        if (!shootingQueue.isEmpty())
                                                            shootingQueue.remove();
                                                    }),
                                                    new ConditionalCommand(
                                                            spindexer.rotateArtifactToShoot(shootingQueue),
                                                            new SequentialCommandGroup(
                                                                    shooter.setState(ShooterState.IDLE),
                                                                    intake.setState(IntakeState.IDLE),
                                                                    setRobotState(RobotState.IDLE)
                                                            ),
                                                            () -> !spindexer.isEmpty() && !shootingQueue.isEmpty()
                                                    )
                                            )
                                    ).interruptOn(() -> spindexer.isEmpty())
                            )
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
                    .whenActive(
                            new ConditionalCommand(
                                    shooter.setState(ShooterState.SHOOT),
                                    shooter.setState(ShooterState.IDLE),
                                    () -> spindexer.isFull())
                    );
        }

        // INTAKE STATE LOGIC
        {
            //When we enter intake state, make sure we rotate an empty slot to the intake and start the intake spinning
            isINTAKE
                    .whenActive(spindexer.rotateEmptyToIntake())
                    .whenActive(intake.setState(IntakeState.INTAKE));

            //If we have an empty slot ready, continually check for artifacts
            isINTAKE
                    .and(new Trigger(spindexer::atIntakeSetpoint))
                    .and(new Trigger(() -> intake.getSensedArtifact() != Artifact.EMPTY))
                    .whenActive(new InstantCommand(() -> justIntaked = true))
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
                    new InstantCommand(() -> {
                        shootingQueue.clear(); //Clear just in case
                        shootingQueue.addAll(spindexer.getRapidShootList());
                    }).andThen(new ConditionalCommand(setRobotState(RobotState.SHOOT), setRobotState(RobotState.IDLE), () -> !shootingQueue.isEmpty()))
            );
        }

        // PRE_PATTERN STATE LOGIC
        {
            isPRE_PATTERN.whenActive(
                    new InstantCommand(() -> {
                        shootingQueue.clear(); //Clear just in case
                        shootingQueue.addAll(PatternMode.getPatternSequence(patternIndexOffset, spindexer.greenCount(), spindexer.purpleCount()));
                    }).andThen(new ConditionalCommand(setRobotState(RobotState.SHOOT), setRobotState(RobotState.IDLE), () -> !shootingQueue.isEmpty()))
            );
        }

        // SHOOT STATE LOGIC
        {
            isSHOOT
                    .and(new Trigger(() -> spindexer.isEmpty() || shootingQueue.isEmpty()))
                    .whenActive(shooter.setState(ShooterState.IDLE))
                    .whenActive(intake.setState(IntakeState.IDLE))
                    .whenActive(setRobotState(RobotState.IDLE));

            isSHOOT
                    .whenActive(spindexer.rotateArtifactToShoot(shootingQueue))
                    .whenActive(shooter.setState(ShooterState.SHOOT));

            isSHOOT
                    .and(new Trigger(() -> shooter.isReadyToShoot()))
                    .whenActive(
                            new SequentialCommandGroup(
                                    new WaitUntilCommand(shooter::atVelocity).withTimeout(7000),
                                    new RepeatCommand(
                                            new SequentialCommandGroup(
                                                    new WaitUntilCommand(spindexer::atSetpoint).withTimeout(250),
                                                    new WaitUntilCommand(shooter::atVelocity).withTimeout(500),
                                                    new WaitCommand(500),
                                                    intake.setState(IntakeState.TRANSFER),
                                                    new WaitCommand(LOADER_TRAVEL_TIME_MS),
                                                    intake.setState(IntakeState.OUTTAKE),
                                                    new WaitCommand(200),
                                                    new InstantCommand(() -> {
                                                        spindexer.shootArtifact();
                                                        if (!shootingQueue.isEmpty())
                                                            shootingQueue.remove();
                                                    }),
                                                    new ConditionalCommand(
                                                            spindexer.rotateArtifactToShoot(shootingQueue),
                                                            new SequentialCommandGroup(
                                                                    shooter.setState(ShooterState.IDLE),
                                                                    intake.setState(IntakeState.IDLE),
                                                                    setRobotState(RobotState.IDLE)
                                                            ),
                                                            () -> !spindexer.isEmpty() && !shootingQueue.isEmpty()
                                                    )
                                            )
                                    ).interruptOn(() -> spindexer.isEmpty())
                            )
                    );
        }
    }

    /**
     * Forces the robot into its IDLE state, hopefully fixing whatever garbage is happening
     */
    public Command forceIdle() {
        return new SequentialCommandGroup(
                setRobotState(RobotState.IDLE),
                new InstantCommand(() -> shootingQueue.clear()),
                intake.setState(IntakeState.IDLE),
                shooter.setState(ShooterState.IDLE),
                new InstantCommand(() -> {
                    try { CommandScheduler.getInstance().cancelAll(); }
                    catch (Exception ignored) {
                        // ! Only for testing today, then remove if it works (which it won't)
                        addPersistentTelemetry("forceIdle threw an error (caught)");
                    }
                })
        );
    }

    private void addPersistentTelemetry(String data) {
        persistentTelemetry.add(data);
    }

    private void printTelemetry() {
        telemetry.addData("State", robotState);

        Pose robotPose = PoseEstimator.getInstance().getRobotPose();
        telemetry.addLine("Pose " + String.format("(%.2f, %.2f, %.2f°)", robotPose.getX(), robotPose.getY(), AngleUnit.RADIANS.toDegrees(robotPose.getHeading())));

        telemetry.addData("Motif", Motif.getMotif());

        Artifact[] artifactsInRobot = spindexer.getCurrentArtifacts();
        telemetry.addLine(String.format("Artifacts In Robot [%s, %s, %s]", artifactsInRobot[0], artifactsInRobot[1], artifactsInRobot[2]));

        String patternStart = (patternIndexOffset == 0) ? "First" : (patternIndexOffset == 1) ? "Middle" : "Last";
        telemetry.addData("Pattern Start", patternStart);

        telemetry.addData("Limelight FPS", vision.getFPS());
        telemetry.addLine(String.format("Goal (Visible = %b | Distance From Robot = %.1f in)", vision.seesGoal(), PoseEstimator.getInstance().getDistanceFromGoal()));

        telemetry.addData("Intake Sensor", intake.getSensedArtifact());

        telemetry.addLine(String.format("Shooter Velocity = %.1f rpm | Shooter Target = %.1f rpm", shooter.getVelocityRPM(), shooter.getTargetRPM()));

        telemetry.addLine(String.format("Offset From Apriltag = %.2f°", turret.getOffsetFromTag()));

        telemetry.addData("Shooting Queue", shootingQueue);

        telemetry.addLine();
        telemetry.addLine("----Subsystem States----");
        telemetry.addData("Intake State", intake.getState());
        telemetry.addData("Turret State", turret.getState());
        telemetry.addData("Shooter State", shooter.getState());
        telemetry.addData("LED State", led.getState());

        telemetry.addLine();
        telemetry.addLine("----Persistent Telemetry----");
        for (String data : persistentTelemetry) {
            telemetry.addLine(data);
        }

        telemetry.addData("auto to teleop turret ticks", AutoToTeleopData.turretTicks);
    }

    private void updateLEDState() {
        if (shooter.atVelocity() && shooter.isReadyToShoot()) {
            led.setState(LEDState.GREEN);
        }
        else if (!shooter.atVelocity() && shooter.isReadyToShoot()) {
            led.setState(LEDState.YELLOW);
        }
        else led.setState(LEDState.RED);
    }

    private void updateTurretState() {
        if (intakeTrigger != null) {
//            if (turret.getState() != TurretState.IDLE && turret.getState() != TurretState.CONSTANT) {
//                if (vision.seesGoal()) {
//                    turret.switchToApriltagControl();
//                    turret.updateOffsetFromTag(vision.getTagOffset());
//                }
//                else {
//                    turret.switchToOdometryControl();
//                    turret.updateOffsetFromTag(0);
//                }
//            }

            if (turret.getState() != TurretState.IDLE && turret.getState() != TurretState.CONSTANT) {
                turret.switchToOdometryControl();
                turret.updateOffsetFromTag(0);
            }
        }
    }

    @Override
    public void periodic() {
        updateLEDState();
        updateTurretState();

        boolean isAuto = (intakeTrigger == null);
        if (isAuto) {
            AutoToTeleopData.spindexerTicks = spindexer.getTicks();
            AutoToTeleopData.turretTicks = turret.getRawTicks();

            Artifact[] artifacts = spindexer.getCurrentArtifacts();
            AutoToTeleopData.slot1Artifact = artifacts[0];
            AutoToTeleopData.slot2Artifact = artifacts[1];
            AutoToTeleopData.slot3Artifact = artifacts[2];

            AutoToTeleopData.endOfAutoPose = PoseEstimator.getInstance().getRobotPose();
        }

        printTelemetry();
    }
}