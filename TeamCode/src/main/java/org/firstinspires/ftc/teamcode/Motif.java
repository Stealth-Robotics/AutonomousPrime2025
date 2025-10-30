package org.firstinspires.ftc.teamcode;

public class Motif {
    private static MotifType motif = MotifType.NULL;

    public static MotifType getMotif() {
        return motif;
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
