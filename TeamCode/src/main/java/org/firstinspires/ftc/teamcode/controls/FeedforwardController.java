package org.firstinspires.ftc.teamcode.controls;

import org.firstinspires.ftc.teamcode.control.motion.State;

public class LinearFeedforwardController implements FeedforwardController {
    private State targetState;
    private double kP; // proportional gain
    private double kV; // velocity gain

    public LinearFeedforwardController(double kP, double kV) {
        this.kP = kP;
        this.kV = kV;
    }

    @Override
    public void setTarget(State target) {
        this.targetState = target;
    }

    @Override
    public double calculateOutput() {
        if (targetState == null) {
            return 0; // no target 0 output
        }

        // calculate control output based on target state
        double output = kP * targetState.getPosition() + kV * targetState.getVelocity();
        return output;
    }

    @Override
    public void reset() {
        targetState = null; // reset the target state
    }

    @Override
    public void setProportionalGain(double kP) {
        this.kP = kP;
    }

    @Override
    public double getProportionalGain() {
        return kP;
    }

    @Override
    public void setVelocityGain(double kV) {
        this.kV = kV;
    }

    @Override
    public double getVelocityGain() {
        return kV;
    }
}