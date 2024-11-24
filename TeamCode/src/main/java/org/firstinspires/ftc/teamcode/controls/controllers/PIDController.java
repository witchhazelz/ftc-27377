package org.firstinspires.ftc.teamcode.controls.controllers;

import com.qualcomm.robotcore.util.ElapsedTime;

public class PIDController {
    private double Kp;
    private double Ki;
    private double Kd;
    private double reference;
    private double lastError;
    private double integralSum;
    private double maxIntegralSum;
    private ElapsedTime timer;

    public PIDController(double Kp, double Ki, double Kd, double maxIntegralSum) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.maxIntegralSum = maxIntegralSum;
        this.integralSum = 0;
        this.lastError = 0;
        this.timer = new ElapsedTime();
    }

    public void setReference(double reference) {
        this.reference = reference;
        integralSum = 0; // Reset integral sum on reference change
    }

    public double calculate(double currentPosition, double derivative) {
        double error = reference - currentPosition;
        integralSum += error * timer.seconds();

        if (integralSum > maxIntegralSum) {
            integralSum = maxIntegralSum;
        }
        if (integralSum < -maxIntegralSum) {
            integralSum = -maxIntegralSum;
        }

        double output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);
        lastError = error;
        timer.reset();

        return output;
    }
}
