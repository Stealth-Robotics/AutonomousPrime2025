package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;

import java.util.function.DoubleSupplier;

public class PoseSupplier {
    DoubleSupplier x;
    DoubleSupplier y;
    DoubleSupplier heading;
    public PoseSupplier(DoubleSupplier x, DoubleSupplier y, DoubleSupplier heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }
    public Pose getAsPose() {
        return new Pose(x.getAsDouble(), y.getAsDouble(), heading.getAsDouble());
    }
}
