package org.firstinspires.ftc.teamcode;

public class Motif {
    private static MotifType motif = MotifType.NULL;

    public static MotifType getMotif() {
        return motif;
    }

    public static Artifact[] getPattern() {
        if (motif == MotifType.PPG) return new Artifact[] {Artifact.PURPLE, Artifact.PURPLE, Artifact.GREEN};
        if (motif == MotifType.PGP) return new Artifact[] {Artifact.PURPLE, Artifact.GREEN, Artifact.PURPLE};
        return new Artifact[] {Artifact.GREEN, Artifact.PURPLE, Artifact.PURPLE};
    }

    public static void setMotif(MotifType detectedMotif) {
        motif = detectedMotif;
    }

    public enum MotifType {
        PPG,
        PGP,
        GPP,
        NULL
    }
}
