package org.firstinspires.ftc.teamcode.controls.controllers;

public class FeedforwardController {
    private double Kv;
    private double Ka;
    private double Kcos;
    private double Kg;

    public FeedforwardController(double Kv, double Ka, double Kcos, double Kg) {
        this.Kv = Kv;
        this.Ka = Ka;
        this.Kcos = Kcos;
        this.Kg = Kg;
    }

    public double calculateFeedforward(double targetVelocity, double targetAcceleration) {
        return (Kv * targetVelocity) + (Ka * targetAcceleration);
    }

    public double calculateGravityCompensation(double targetAngle) {
        return Math.cos(Math.toRadians(targetAngle)) * Kcos;
    }

    public double calculateSlideCompensation() {
        return Kg;
    }

    public double calculateTotalOutput(double targetVelocity, double targetAcceleration, double targetAngle) {
        double feedforwardOutput = calculateFeedforward(targetVelocity, targetAcceleration);
        double gravityOutput = calculateGravityCompensation(targetAngle);
        return feedforwardOutput + gravityOutput;
    }
}
