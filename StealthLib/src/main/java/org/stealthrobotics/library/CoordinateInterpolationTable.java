package org.stealthrobotics.library;

import com.pedropathing.geometry.Pose;

import java.util.ArrayList;
import java.util.List;

public class CoordinateInterpolationTable {
    private final List<CoordinateValue> points = new ArrayList<>();
    private final double idwPower;

    private static class CoordinateValue {
        public final double x;
        public final double y;
        public final double value;

        public CoordinateValue(double x, double y, double value) {
            this.x = x;
            this.y = y;
            this.value = value;
        }
    }

    /**
     * @param idwPower The influence of closer data points vs farther ones (larger values favor closer data)
     **/
    public CoordinateInterpolationTable(double idwPower) {
        this.idwPower = idwPower;
    }

    public void addPoint(double x, double y, double value) {
        points.add(new CoordinateValue(x, y, value));
    }

    public double get(double x, double y) {
        if (points.isEmpty()) return 0.0;

        double weightedSum = 0;
        double weightTotal = 0;

        for (CoordinateValue p : points) {
            double dx = x - p.x;
            double dy = y - p.y;
            double dist = Math.sqrt(dx*dx + dy*dy);

            if (dist == 0) return p.value;

            double weight = 1.0 / Math.pow(dist, idwPower);

            weightedSum += weight * p.value;
            weightTotal += weight;
        }

        return weightedSum / weightTotal;
    }
}
