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
    private double velocityTolerance;

    private double lastMeasuredValue;
    private double lastTime;

    public AnglePIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.lastTime = (double) System.nanoTime() / 1E9;
    }

    public AnglePIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = 0.0;
        this.lastTime = (double) System.nanoTime() / 1E9;
    }

    public void setPositionTolerance(double newTolerance) {
        tolerance = newTolerance;
    }

    public void setVelocityTolerance(double velocityTolerance) {
        this.velocityTolerance = velocityTolerance;
    }

    public void setSetPoint(double setPoint) {
        reference = setPoint;
    }

    public double getSetPoint() {
        return reference;
    }

    public double getVelocityError() {
        return (measuredValue - lastMeasuredValue) / (getTimeSinceLastUpdate());
    }

    public double getError() {
        return wrapAngle(reference - measuredValue);
    }

    public boolean atSetPoint() {
        double positionError = Math.abs(Math.abs(reference) - Math.abs(measuredValue));
        double velocityError = Math.abs(Math.abs(measuredValue) - Math.abs(lastMeasuredValue)) / (getTimeSinceLastUpdate());

        return positionError <= tolerance && velocityError <= velocityTolerance;
    }

    public double calculate(double measuredValue) {
        this.measuredValue = measuredValue;

        double time = (double) System.nanoTime() / 1E9;
        double error = getError();
        double derivative = (error - lastError) / (time - lastTime);

        integralSum += error * (time - lastTime);

        lastError = error;
        lastMeasuredValue = measuredValue;
        lastTime = time;

        return (kP * error) + (kI * integralSum) + (kD * derivative) + kF;
    }

    private double wrapAngle(double theta) {
        return AngleUnit.normalizeDegrees(theta);
    }

    private double getTimeSinceLastUpdate() {
        return (double) System.nanoTime() / 1E9 - lastTime;
    }
}
