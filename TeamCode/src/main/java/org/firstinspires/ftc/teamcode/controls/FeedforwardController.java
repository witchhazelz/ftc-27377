package org.firstinspires.ftc.teamcode.control.controllers;

import org.firstinspires.ftc.teamcode.control.motion.State;

public class FeedforwardController implements Controller {
    private State targetState; // desired target state
    private double kP; // proportional gain
    private double kV; // velocity gain
    private double kA; // acceleration gain

    public FeedforwardController(double kP, double kV, double kA) {
        this.kP = kP;
        this.kV = kV;
        this.kA = kA;
    }

    @Override
    public void setTarget(State target) {
        this.targetState = target;
    }

    @Override
    public double calculateOutput(State currentState) {
        if (targetState == null) {
            return 0; // no target set=> return zero output
        }

        // calculate control output based on target state and current state
        double output = (kP * (targetState.getVelocity() - currentState.getVelocity())) +
                (kV * targetState.getVelocity()) +
                (kA * targetState.getAcceleration());

        return output;
    }

    @Override
    public void reset() {
        targetState = null; // reset target state
    }

    // set and get for gains
    public void setProportionalGain(double kP) {
        this.kP = kP;
    }

    public double getProportionalGain() {
        return kP;
    }

    public void setVelocityGain(double kV) {
        this.kV = kV;
    }

    public double getVelocityGain() {
        return kV;
    }

    public void setAccelerationGain(double kA) {
        this.kA = kA;
    }

    public double getAccelerationGain() {
        return kA;
    }
}