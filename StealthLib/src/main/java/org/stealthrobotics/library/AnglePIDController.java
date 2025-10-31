package org.stealthrobotics.library;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AnglePIDController {
    private final double kP;
    private final double kI;
    private final double kD;
    private final double kF;

    private double reference;
    private double measuredValue;

    private double integralSum;
    private double lastError;

    private double tolerance;

    private double lastTime;

    public AnglePIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = 0;
        this.lastTime = (double) System.nanoTime() / 1E9;
    }

    public AnglePIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.lastTime = (double) System.nanoTime() / 1E9;
    }

    public void setPositionTolerance(double newTolerance) {
        tolerance = newTolerance;
    }

    public void setSetPoint(double setPoint) {
        reference = wrapAngle(setPoint);
    }

    public double getSetPoint() {
        return reference;
    }

    public double getError() {
        return wrapAngle(reference - measuredValue);
    }

    public boolean atSetPoint() {
        double positionError = Math.abs(getError());
        return positionError <= tolerance;
    }

    public double calculate(double measuredValue) {
        this.measuredValue = measuredValue;

        double time = (double) System.nanoTime() / 1E9;
        double error = getError();
        double derivative = (error - lastError) / (time - lastTime);

        integralSum += error * (time - lastTime);

        lastError = error;
        lastTime = time;

        return (kP * error) + (kI * integralSum) + (kD * derivative) + ((-Math.signum(error)) * kF);
    }

    private double wrapAngle(double theta) {
        return AngleUnit.normalizeDegrees(theta);
    }
}
