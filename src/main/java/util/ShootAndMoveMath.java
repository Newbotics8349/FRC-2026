package util;

import frc.robot.Constants;

public class ShootAndMoveMath {
    private final Vector3 d;
    private final Vector3 vr;
    private CorrectTime time;

    public ShootAndMoveMath(Vector3 d, Vector3 vr) {
        this.d = d;
        this.vr = vr;

        this.time = getTime(d, vr);
    }

    private CorrectTime getTime(Vector3 d, Vector3 vr, double t, int iterations) {
        double newTime = t - timeEquation(t) / timeDerivative(t);
        if (Math.abs(timeEquation(newTime)) < 0.1 || iterations > 5) {
            return new CorrectTime(newTime, iterations > 5);
        }
        return getTime(d, vr, newTime, iterations + 1);
    }

    private CorrectTime getTime(Vector3 d, Vector3 vr) {
        return getTime(d, vr, Math.PI / 2, 0);
    }

    private double timeEquation(double t) {
        return d.z / t + 0.5 * Constants.ShooterConstants.g * t - Math.sqrt(Math.pow(Constants.ShooterConstants.vout, 2) - Math.pow(d.x / t - vr.x, 2) - Math.pow(d.y / t - vr.y, 2));
    }

    private double timeDerivative(double t) {
        return -1 * d.z / Math.pow(t, 2) + 0.5 * Constants.ShooterConstants.g - 0.5 * Math.pow(Math.pow(Constants.ShooterConstants.vout, 2) - Math.pow(d.x / t  - vr.x, 2) - Math.pow(d.y / t - vr.y, 2), -0.5) * (2 * (d.x / t - vr.x) * (d.x / Math.pow(t, 2)) + 2 * (d.y / t - vr.y) * (d.y / Math.pow(t, 2)));
    }

    public double rotToTarget() {
        if (time.highIterations || !time.correctRange) return 500;

        Vector3 v = new Vector3(d.x / time.time - vr.x, d.y / time.time - vr.y, 0);
        Vector3 u = new Vector3(0, 1, 0);
        return Math.acos(dotProduct(u, v) / magnitude(v));
    }

    public double heightToTarget() {
        if (time.highIterations || !time.correctRange) return 500;

        Vector3 v = new Vector3(d.x / time.time - vr.x, d.y / time.time - vr.y, d.z / time.time + 0.5 * Constants.ShooterConstants.g * time.time);
        Vector3 u = new Vector3(d.x / time.time - vr.x, d.y / time.time - vr.y, 0);
        return Math.acos(dotProduct(u, v) / (magnitude(u) * magnitude(v)));
    }

    private double dotProduct(Vector3 u, Vector3 v) {
        return u.x * v.x + u.y * v.y + u.z * v.z;
    }

    private double magnitude(Vector3 v) {
        return Math.sqrt(Math.pow(v.x, 2) + Math.pow(v.y, 2) + Math.pow(v.z, 2));
    }
    
    private class CorrectTime {
        public double time;
        public boolean highIterations;
        public boolean correctRange;

        public CorrectTime(double time, boolean highIterations) {
            this.time = time;
            this.highIterations = highIterations;
            this.correctRange = 0 < time;
        }
    }
}