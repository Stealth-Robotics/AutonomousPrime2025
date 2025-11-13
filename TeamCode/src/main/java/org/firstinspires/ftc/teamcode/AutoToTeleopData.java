package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class AutoToTeleopData {
    /*
        Set any desired teleop subsystem setpoints here and they will be overwritten if auto is run
        If auto isn't run they will act as the testing values for teleop
     */

    public static Pose2D pinpointPose = new Pose2D(DistanceUnit.INCH, 72, 72, AngleUnit.DEGREES, 90);
    public static double spindexerTicks = 0.0;
    public static double turretTicks = 0.0;

    public static Artifact slot1Artifact = Artifact.GREEN;
    public static Artifact slot2Artifact = Artifact.EMPTY;
    public static Artifact slot3Artifact = Artifact.EMPTY;
}
