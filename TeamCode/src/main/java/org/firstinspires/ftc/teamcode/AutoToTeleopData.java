package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.enums.Artifact;

public class AutoToTeleopData {
    /*
        Set any desired teleop subsystem setpoints here and they will be overwritten if auto is run
        If auto isn't run they will act as the testing values for teleop
     */

    public static double spindexerTicks = 0.0;
    public static double turretTicks = 0.0;

    public static Artifact slot1Artifact = Artifact.EMPTY;
    public static Artifact slot2Artifact = Artifact.EMPTY;
    public static Artifact slot3Artifact = Artifact.EMPTY;

    public static Pose endOfAutoPose = new Pose(0, 0, Math.toRadians(0));
}
