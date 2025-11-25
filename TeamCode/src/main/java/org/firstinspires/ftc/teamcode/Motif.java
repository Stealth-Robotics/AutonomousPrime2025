package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.enums.Artifact;
import org.firstinspires.ftc.teamcode.enums.MotifType;

import java.util.ArrayList;

public class Motif {
    private static MotifType motif = MotifType.PGP; //Default value (in case camera fails)

    public static MotifType getMotif() {
        return motif;
    }

    public static ArrayList<Artifact> getPatternList() {
        ArrayList<Artifact> patternList = new ArrayList<>();
        if (motif == MotifType.PPG) {
            patternList.add(Artifact.PURPLE);
            patternList.add(Artifact.PURPLE);
            patternList.add(Artifact.GREEN);
            return patternList;
        }
        else if (motif == MotifType.PGP) {
            patternList.add(Artifact.PURPLE);
            patternList.add(Artifact.GREEN);
            patternList.add(Artifact.PURPLE);
            return patternList;
        }
        else {
            patternList.add(Artifact.GREEN);
            patternList.add(Artifact.PURPLE);
            patternList.add(Artifact.PURPLE);
            return patternList;
        }
    }

    public static void setMotif(MotifType detectedMotif) {
        motif = detectedMotif;
    }
}
