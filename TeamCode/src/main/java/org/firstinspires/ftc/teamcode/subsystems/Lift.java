package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.control.controller.PIDController;
import org.firstinspires.ftc.teamcode.controls.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.controls.motion.State;

public class Lift {

    // PID gains and constants
    public static PIDGains pidGains = new PIDGains(0.5, 0.4, 0.01, 1.0); // Tune gains here
    private static final double kG = 0.1;                      // Gravity compensation
    private static final double MAX_VOLTAGE = 13.0;            // Fully charged battery voltage
    private static final double INCHES_PER_TICK = 0.001;       // Encoder conversion factor (example)

    // Motor and sensor
    private final DcMotorEx liftMotor;
    private final VoltageSensor batteryVoltageSensor;
    private final PIDController controller = new PIDController();

    // Position tracking
    private double targetPosition = 0.0;  // Target position in inches
    private double currentPosition = 0.0; // Current position in inches
    private double manualPower = 0.0;

    // Timer for debugging/tuning
    private final ElapsedTime timer = new ElapsedTime();

    public Lift(HardwareMap hardwareMap) {
        // Initialize hardware
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set up PID gains
        controller.setGains(pidGains);
    }

    /**
     * Move the lift to a target position in inches using PID control.
     * @param targetPositionInches Target position in inches.
     */
    public void moveToPosition(double targetPositionInches) {
        this.targetPosition = targetPositionInches;
    }

    /**
     * Run the lift system (should be called repeatedly in a loop).
     */
    public void run() {
        // Read the current position in inches using encoder ticks
        currentPosition = liftMotor.getCurrentPosition() * INCHES_PER_TICK;

        // Set PID target
        controller.setTarget(new State(targetPosition));

        // Calculate PID output
        double pidOutput = controller.calculate(new State(currentPosition));

        // Apply gravity compensation
        double voltageCompensation = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double totalOutput = (pidOutput + kG) * voltageCompensation;

        // Clamp the motor power between -1.0 and 1.0
        totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));

        // Apply power to motor
        liftMotor.setPower(totalOutput);
    }

    /**
     * Run the lift manually with specified power.
     * @param power Motor power (-1.0 to 1.0).
     */
    public void runManual(double power) {
        manualPower = power;
        liftMotor.setPower(manualPower);
    }

    /**
     * Stops the lift motor.
     */
    public void stop() {
        liftMotor.setPower(0.0);
    }

    /**
     * Get the current position of the lift in inches.
     */
    public double getPosition() {
        return currentPosition;
    }

    /**
     * Debugging method to print telemetry data.
     */
    public void printTelemetry() {
        System.out.println("Target Position: " + targetPosition + " inches");
        System.out.println("Current Position: " + currentPosition + " inches");
        System.out.println("PID Output: " + controller.calculate(new State(currentPosition)));
        System.out.println("Elapsed Time: " + timer.seconds() + " seconds");
    }
}
