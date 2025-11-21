package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

public enum Artifact {
    EMPTY,
    GREEN,
    PURPLE;

    @NonNull
    @Override
    public String toString() {
        return (this == EMPTY) ? "EMPTY" : (this == GREEN) ? "GREEN" : "PURPLE";
    }
}
