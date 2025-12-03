package org.stealthrobotics.library;

public class SquIDController {
    private double p = 0;
    private double setpoint = 0;

    public SquIDController(double p) {
        this.p = p;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public double calculate(double currentPosition, double targetPosition) {
        double error = targetPosition - currentPosition;
        return signedSqrt(error) * p;
    }

    public double calculate(double currentPosition) {
        double error = setpoint - currentPosition;
        return signedSqrt(error) * p;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setP(double p) {
        this.p = p;
    }

    private static double signedSqrt(double val) {
        return Math.signum(val) * Math.sqrt(Math.abs(val));
    }
}