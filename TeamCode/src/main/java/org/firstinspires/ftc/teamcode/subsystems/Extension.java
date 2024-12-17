package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controls.controllers.PIDController;
import org.firstinspires.ftc.teamcode.controls.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.controls.motion.State;

public class Extension {

    private static final PIDGains pidGains = new PIDGains(0.5, 0.4, 0.01, 1.0); // PID gains
    private static final double kG = 0.1; // gravity offset constant
    private static final double MAX_VOLTAGE = 13.0; // max battery voltage
    private static final double INCHES_PER_TICK = 0.008; // conversion factor

    private final DcMotorEx leftSlideMotor;
    private final DcMotorEx rightSlideMotor;
    private final VoltageSensor batteryVoltageSensor;

    private final PIDController leftController = new PIDController();
    private final PIDController rightController = new PIDController();

    private double leftTargetPosition = 0.0; // target position for left slide
    private double rightTargetPosition = 0.0; // target position for right slide

    private double leftCurrentPosition = 0.0;
    private double rightCurrentPosition = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    // Constructor
    public Extension(HardwareMap hardwareMap) {
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // DO NOT DO THIS THING EVER. DO NOT RUN IT AT ALL.
        //leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Initialize PID controllers
        leftController.setGains(pidGains);
        rightController.setGains(pidGains);
    }

    // Set target position in inches
    public void moveToPosition(double targetPositionInches) {
        this.leftTargetPosition = targetPositionInches;
        this.rightTargetPosition = targetPositionInches; // keep both motors synchronized
    }

    // PID-controlled movement
    public void run() {
        // Read current position in inches
        leftCurrentPosition = leftSlideMotor.getCurrentPosition() * INCHES_PER_TICK;
        rightCurrentPosition = rightSlideMotor.getCurrentPosition() * INCHES_PER_TICK;

        // Update PID controller targets
        leftController.setTarget(new State(leftTargetPosition));
        rightController.setTarget(new State(rightTargetPosition));

        // Calculate PID outputs
        double leftPidOutput = leftController.calculate(new State(leftCurrentPosition));
        double rightPidOutput = rightController.calculate(new State(rightCurrentPosition));

        // Voltage compensation
        double voltageCompensation = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();

        // calculate total motor power with gravity compensation
        double leftTotalOutput = (leftPidOutput + kG) * voltageCompensation;
        double rightTotalOutput = (rightPidOutput + kG) * voltageCompensation;

        // clamp power outputs to [-1.0, 1.0]
        leftTotalOutput = Math.max(-1.0, Math.min(1.0, leftTotalOutput));
        rightTotalOutput = Math.max(-1.0, Math.min(1.0, rightTotalOutput));

        // Set motor powers
        leftSlideMotor.setPower(leftTotalOutput);
        rightSlideMotor.setPower(rightTotalOutput);
    }

    // stop both slides
    public void stop() {
        leftSlideMotor.setPower(0.0);
        rightSlideMotor.setPower(0.0);
    }

    // Manual control mode
    public void runManual(double power) {
        leftSlideMotor.setPower(power);
        rightSlideMotor.setPower(power);
    }

    // Telemetry for debugging
    public void printTelemetry() {
        System.out.println("Left Target Position: " + leftTargetPosition + " inches");
        System.out.println("Left Current Position: " + leftCurrentPosition + " inches");
        System.out.println("Right Target Position: " + rightTargetPosition + " inches");
        System.out.println("Right Current Position: " + rightCurrentPosition + " inches");
    }

    // getter for left slide position
    public double getLeftPosition() {
        return leftCurrentPosition;
    }

    // getter for right slide position
    public double getRightPosition() {
        return rightCurrentPosition;
    }
}


