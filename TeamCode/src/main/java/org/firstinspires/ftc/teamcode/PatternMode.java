package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.enums.Artifact;

import java.util.ArrayList;
import java.util.LinkedList;

public class PatternMode {
    /**
     * @param patternStartOffset where in the pattern should be start
     * @param gCount number of green artifacts available
     * @param pCount number of purple artifacts available
     * @return A list of the exact sequence that should be shot
     */
    public static ArrayList<Artifact> getPatternSequence(int patternStartOffset, int gCount, int pCount) {
        ArrayList<Artifact> patternList = Motif.getPatternList();
        ArrayList<Artifact> patternSequence = new ArrayList<>();

        //Works with the number balls available to shoot
        int totalArtifacts = (gCount + pCount);
        for (int i = 0; i < totalArtifacts; i++) {
            //Essentially wraps around the pattern based on the desired starting index (to complete partially completed patterns)
            Artifact nextInSequence = patternList.get((i + patternStartOffset) % 3);
            if (nextInSequence == Artifact.GREEN && gCount > 0) {
                patternSequence.add(nextInSequence);
                gCount--;
            }
            else if (nextInSequence == Artifact.PURPLE && pCount > 0) {
                patternSequence.add(nextInSequence);
                pCount--;
            }
            else {
                return patternSequence; //Stop and continue with the pattern so far (don't want to ruin the motif)
            }
        }

        return patternSequence;
    }
}

