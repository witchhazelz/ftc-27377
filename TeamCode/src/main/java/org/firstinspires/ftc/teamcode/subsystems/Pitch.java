package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx; // Use DcMotorEx for advanced motor control
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Pitch {
    // Constants
    private static final double kG = 0.1; // Gravity compensation constant
    private static final double MAX_VOLTAGE = 13.0;
    private static final double TICKS_PER_REV = 1440.0; // Example value, adjust based on encoder specs
    private static final double SPROCKET_RADIUS = 1.0; // Radius in inches
    private static final double RADIANS_PER_TICK = (2 * Math.PI) / TICKS_PER_REV;
    private static final double INCHES_PER_RADIAN = SPROCKET_RADIUS; // Linear distance per radian

    // Hardware and controller
    private final DcMotorEx liftMotor; // Use DcMotorEx for advanced motor control
    private final VoltageSensor batteryVoltageSensor;

    // PID variables
    private double kP = 0.005; // Proportional gain
    private double kI = 0.002; // Integral gain
    private double kD = 0.0;   // Derivative gain
    private double targetPositionRadians = 0.0;
    private double currentPositionRadians = 0.0;
    private double errorIntegral = 0.0;
    private double lastError = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    // Define constant power values for incremental movement
    private static final double LIFT_UP_POWER = 0.5; // Power to move up
    private static final double LIFT_DOWN_POWER = -0.5; // Power to move down

    // Constructor
    public Pitch(HardwareMap hardwareMap) {
        // Initialize hardware
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor"); // Use DcMotorEx for advanced control
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public void moveToPosition(double targetPositionInches) {
        // Convert linear target position to radians
        this.targetPositionRadians = targetPositionInches / INCHES_PER_RADIAN;
    }

    public void run() {
        // Read current position in radians
        currentPositionRadians = liftMotor.getCurrentPosition() * RADIANS_PER_TICK;

        // Calculate PID output
        double error = targetPositionRadians - currentPositionRadians;
        errorIntegral += error * timer.seconds();
        double errorDerivative = (error - lastError) / timer.seconds();
        lastError = error;

        double pidOutput = (kP * error) + (kI * errorIntegral) + (kD * errorDerivative);

        // Apply gravity compensation and voltage scaling
        double voltageCompensation = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double totalOutput = (pidOutput + kG) * voltageCompensation;

        // Clamp motor power between -1.0 and 1.0
        totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));

        // Reverse the output power if necessary (maybe??)
        totalOutput = -totalOutput; // Reverse the direction if needed

        // Apply power to motor
        liftMotor.setPower(totalOutput);
    }

    public void controlLift(Gamepad gamepad) {
        double leftTrigger = gamepad.left_trigger; // Get the value of the left trigger
        double rightTrigger = gamepad.right_trigger; // Get the value of the right trigger

        if (leftTrigger > 0.1) { // If the left trigger is pressed
            runManual(LIFT_UP_POWER); // Move up at a constant power
        } else if (rightTrigger > 0.1) { // If the right trigger is pressed
            runManual(LIFT_DOWN_POWER); // Move down at a constant power
        } else {
            stop(); // Stop lift when no trigger is pressed
        }
    }

    private void runManual(double power) {
        liftMotor.setPower(power); // Set the motor power
    }

    private void stop() {
        liftMotor.setPower(0); // Stop the motor
    }
}