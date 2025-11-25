package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public class PatternMode {
    public static Queue<Artifact> getPatternSequence(int patternStartOffset, ArrayList<Artifact> availableBalls) {
        ArrayList<Artifact> patternList = Motif.getPatternList();
        Queue<Artifact> patternSequence = new LinkedList<>();

        //Works with the number balls available to shoot
        for (int i = 0; i < availableBalls.size(); i++) {
            //Essentially wraps around the pattern based on the desired starting index (to complete partially completed patterns)
            Artifact nextInSequence = patternList.get((i - patternStartOffset + 3) % 3);
            if (availableBalls.contains(nextInSequence)) {
                patternSequence.add(nextInSequence);
                availableBalls.remove(nextInSequence);
            }
            else break;
        }
        return patternSequence;
    }
}

