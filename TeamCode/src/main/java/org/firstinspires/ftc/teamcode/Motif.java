package org.firstinspires.ftc.teamcode;

import java.util.LinkedList;
import java.util.Queue;

public class Motif {
    private static MotifType motif = MotifType.NULL;

    public static MotifType getMotif() {
        return motif;
    }

    public static Queue<Artifact> getPatternQueue() {
        Queue<Artifact> queue = new LinkedList<>();
        if (motif == MotifType.PPG) {
            queue.add(Artifact.PURPLE);
            queue.add(Artifact.PURPLE);
            queue.add(Artifact.GREEN);
        }
        else if (motif == MotifType.PGP) {
            queue.add(Artifact.PURPLE);
            queue.add(Artifact.GREEN);
            queue.add(Artifact.PURPLE);
        }
        else {
            queue.add(Artifact.GREEN);
            queue.add(Artifact.PURPLE);
            queue.add(Artifact.PURPLE);
        }

        return queue;
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
