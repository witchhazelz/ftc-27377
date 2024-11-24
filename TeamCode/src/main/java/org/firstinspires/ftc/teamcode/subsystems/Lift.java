package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Lift {
    // motor parameters
    public double maxLiftSpeed; // max speed for lift
    public double liftAcceleration; // acceleration for lift
    public double liftPositionTolerance; // tolerance for position control

    // public enums for lift states
    public enum LiftState {
        IDLE,
        LIFTING,
        LOWERING,
        HOLDING
    }

    //boolean to check if the lift is in motion
    public boolean isMoving;

    private DcMotor motor1; // first motor for the lift
    private DcMotor motor2; // second motor for the lift
    private LiftState currentState; // current state of the lift
    private double targetPosition; // target position for the lift

    // Constructor
    public Lift(DcMotor motor1, DcMotor motor2, double maxLiftSpeed, double liftAcceleration, double liftPositionTolerance) {
        this.motor1 = motor1;
        this.motor2 = motor2;
        this.maxLiftSpeed = maxLiftSpeed;
        this.liftAcceleration = liftAcceleration;
        this.liftPositionTolerance = liftPositionTolerance;
        this.currentState = LiftState.IDLE;
        this.isMoving = false;
        this.targetPosition = 0;

        // set motor directions if needed
        this.motor1.setDirection(DcMotorSimple.Direction.FORWARD);
        this.motor2.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // method to set the target position for the lift
    public void setTargetPosition(double position) {
        this.targetPosition = position;
        this.currentState = LiftState.LIFTING;
        this.isMoving = true;
    }

    // method to update lift's position based on current state
    public void update(double currentPosition) {
        switch (currentState) {
            case LIFTING:
                if (Math.abs(currentPosition - targetPosition) > liftPositionTolerance) {
                    // move motors towards the target position
                    double speed = calculateSpeed(currentPosition);
                    motor1.setPower(speed);
                    motor2.setPower(speed);
                } else {
                    // stop lifting when target is reached
                    stopLift();
                }
                break;

            case LOWERING:
                if (Math.abs(currentPosition - targetPosition) > liftPositionTolerance) {
                    // move motors downwards towards the target position
                    double speed = calculateSpeed(currentPosition);
                    motor1.setPower(-speed); // negative speed for lowering
                    motor2.setPower(-speed); // negative speed for lowering
                } else {
                    stopLift();
                }
                break;

            case HOLDING:
                motor1.setPower(0);
                motor2.setPower(0);
                break;

            case IDLE:
            default:
                stopLift();
                break;
        }
    }

    // stop lift
    public void stopLift() {
        motor1.setPower(0);
        motor2.setPower(0);
        currentState = LiftState.IDLE;
        isMoving = false;
    }

    // calculate speed based on current position (could be more complex)
    private double calculateSpeed(double currentPosition) {
        // linear speed calculation based on distance to target
        double distance = targetPosition - currentPosition;
        double speed = Math.min(maxLiftSpeed, Math.abs(distance)); // make sure speed does not exceed max
        return Math.signum(distance) * speed; // return speed with direction
    }
}
