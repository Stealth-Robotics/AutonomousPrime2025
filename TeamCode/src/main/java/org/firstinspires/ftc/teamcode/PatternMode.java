package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Queue;

public enum PatternMode {
    START_BALL_1,
    START_BALL_2,
    START_BALL_3;


    public static Queue<Artifact> getPatternList(Motif.MotifType motif, PatternMode patternMode, int numBalls){
        //Assigns an integer value to the patternMode
        int offset = (patternMode == START_BALL_1 ? 0 : (patternMode == START_BALL_2 ? 1 : 2));
        //Generates a list of Artifacts according to the motif
        ArrayList<Artifact>  patternList = Motif.getPatternList(motif);
        Queue<Artifact> output = new LinkedList<>();
        //Only adds the needed number of balls
        for(int i = 0; i < numBalls; i++) {
            //Adds the artifacts from the ArrayList to the Queue
            output.add(patternList.get((i - offset + 3) % 3));
            // i - offset is the index of the Artifact
            // + 3 % 3 represents wraparound within the motif
        }
        return output;
    }
}

