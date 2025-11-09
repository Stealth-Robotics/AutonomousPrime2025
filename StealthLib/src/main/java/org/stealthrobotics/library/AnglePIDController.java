package org.stealthrobotics.library;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class AnglePIDController {
    private double kP;
    private double kI;
    private double kD;
    private double kF;

    private double reference;
    private double measuredValue;

    private double integralSum;
    private double lastError;

    private double tolerance;

    private double lastTime;

    private double lastReference;

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
        lastReference = reference;
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

        // reset the integral if the reference is changed.
        if (reference != lastReference) {
            integralSum = 0;
        }

        integralSum += error * (time - lastTime);

        if (integralSum > 10000) {
            integralSum = 10000;
        }
        if (integralSum < -10000) {
            integralSum = -10000;
        }

        lastError = error;
        lastTime = time;

        return (kP * error) + (kI * integralSum) + (kD * derivative) + ((-Math.signum(error)) * kF);
    }

    private double wrapAngle(double theta) {
        return AngleUnit.normalizeDegrees(theta);
    }
}
