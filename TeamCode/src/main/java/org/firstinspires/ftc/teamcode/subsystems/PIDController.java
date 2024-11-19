package org.firstinspires.ftc.teamcode.subsystems;

public class PIDController {
    // constants for PID gains and filter
    public double kP;
    public double kI;
    public double kD;
    public double filterCoefficient;

    // private hardware objects
    private double previousError;
    private double integral;
    private double setpoint;

    // constructor
    public PIDController(double kP, double kI, double kD, double filterCoefficient) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.filterCoefficient = filterCoefficient;
        this.previousError = 0;
        this.integral = 0;
        this.setpoint = 0;
    }

    // method to set desired setpoint
    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    // method to compute pid output
    public double compute(double currentValue) {
        double error = setpoint - currentValue;
        integral += error; // accumulate integral error

        // calculate derivative (with filtering)
        double derivative = error - previousError;
        previousError = error;

        // calculate PID output
        double output = (kP * error) + (kI * integral) + (kD * derivative);
        return output;
    }

    // Method to reset the PID controller
    public void reset() {
        previousError = 0;
        integral = 0;
    }

    // could update gains for live tuning???
    public void updateGains(double newKp, double newKi, double newKd, double newFilterCoefficient) {
        this.kP = newKp;
        this.kI = newKi;
        this.kD = newKd;
        this.filterCoefficient = newFilterCoefficient;
    }
}
