package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.util.function.DoubleSupplier;

public class PoseSupplier {
    private final DoubleSupplier x;
    private final DoubleSupplier y;
    private final DoubleSupplier heading;

    public PoseSupplier(DoubleSupplier x, DoubleSupplier y, DoubleSupplier heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2D getAsPose() {
        return new Pose2D(DistanceUnit.INCH, x.getAsDouble(), y.getAsDouble(), AngleUnit.DEGREES, heading.getAsDouble());
    }
}
