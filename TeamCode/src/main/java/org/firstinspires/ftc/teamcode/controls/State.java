package org.firstinspires.ftc.teamcode.controls;

public class State {
    private double position;
    private double velocity;
    private double acceleration;

    public State(double position, double velocity, double acceleration) {
        this.position = position;
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public double getPosition() {
        return position;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getAcceleration() {
        return acceleration;
    }
}
