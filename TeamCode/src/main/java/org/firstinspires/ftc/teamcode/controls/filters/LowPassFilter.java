package org.firstinspires.ftc.teamcode.controls.filters;
//NEEDS MORE WORK
public class LowPassFilter {
    private double a;
    private double previousEstimate;

    public LowPassFilter(double a) {
        this.a = a;
        this.previousEstimate = 0;
    }

    public double filter(double measurement) {
        double currentEstimate = (a * previousEstimate) + (1 - a) * measurement;
        previousEstimate = currentEstimate;
        return currentEstimate;
    }
}
