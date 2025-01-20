package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

public class Pitch {
    // constants
    private static final double kG = 0.1; // gravity compensation constant
    private static final double MAX_VOLTAGE = 13.0;
    private static final double TICKS_PER_REV = 1440.0; // adjust based on encoder specs
    private static final double SPROCKET_RADIUS = 1.0; // radius in inches
    private static final double RADIANS_PER_TICK = (2 * Math.PI) / TICKS_PER_REV;
    private static final double INCHES_PER_RADIAN = SPROCKET_RADIUS; // linear distance per radian

    // predefined positions for manual override
    private static final double POSITION_A_RADIANS = Math.PI / 4; // predefined pos A (45 degrees)
    private static final double POSITION_X_RADIANS = Math.PI / 2; // predefined pos X (90 degrees)

    // hardware + controller
    private final DcMotorEx liftMotor;
    private final VoltageSensor batteryVoltageSensor;

    // PID variables
    private double kP = 0.005; // how fast I'm going to the target
    private double kI = 0.0001; // how fast I want to build up
    private double kD = 0.0001; // slow down when going too fast
    private double targetPositionRadians = 0.0;
    private double currentPositionRadians = 0.0;
    private double errorIntegral = 0.0;
    private double lastError = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    // power constants for manual movement
    private static final double LIFT_UP_POWER = 0.5;
    private static final double LIFT_DOWN_POWER = -0.5;

    // mass of the object being lifted (adjust as needed)
    private static final double MASS = 1.0; // in kg
    private static final double GRAVITY = 9.81; // acceleration due to gravity in m/s^2

    // constructor
    public Pitch(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void moveToPositionRadians(double targetPositionRadians) {
        this.targetPositionRadians = targetPositionRadians;
    }

    public void run() {
        currentPositionRadians = liftMotor.getCurrentPosition() * RADIANS_PER_TICK;

        // Reset timer each cycle to prevent integral windup
        double deltaTime = timer.seconds();
        timer.reset();

        // PID calculations
        double error = targetPositionRadians - currentPositionRadians;
        errorIntegral = Math.max(-1.0, Math.min(1.0, errorIntegral + (error * deltaTime))); // Clamp integral
        double errorDerivative = (error - lastError) / deltaTime;
        lastError = error;

        double pidOutput = (kP * error) + (kI * errorIntegral) + (kD * errorDerivative);

        // Calculate gravitational force and required power
        double gravitationalForce = MASS * GRAVITY; // in Newtons
        double requiredPower = gravitationalForce * SPROCKET_RADIUS; // in Watts

        // Convert required power to motor power (assuming linear relationship)
        double minPower = requiredPower / MAX_VOLTAGE; // normalize to motor power range

        // Gravity compensation and voltage scaling
        double voltageCompensation = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double totalOutput = (pidOutput + kG) * voltageCompensation;

        // clamp output power
        totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));

        // Locking mechanism
        if (Math.abs(totalOutput) <= minPower) {
            totalOutput = Math.signum(totalOutput) * minPower; // apply minimum power to hold position
        }

        liftMotor.setPower(totalOutput);
    }

    public void controlLift(Gamepad gamepad) {
        if (gamepad.a) {
            moveToPositionRadians(POSITION_A_RADIANS);
            run();
        } else if (gamepad.x) {
            moveToPositionRadians(POSITION_X_RADIANS);
            run();
        } else if (gamepad.left_bumper || gamepad.right_bumper) {
            currentPositionRadians = liftMotor.getCurrentPosition() * RADIANS_PER_TICK;
            double increment = gamepad.left_bumper ? (Math.PI / 36) : -(Math.PI / 36);
            moveToPositionRadians(currentPositionRadians + increment);
            run();
        } else {
            // Hold current position
            moveToPositionRadians(currentPositionRadians);
            run();
        }
    }

    public void runManual(double power) {
        liftMotor.setPower(power);
    }

    public double getMotorRotations() {
        return liftMotor.getCurrentPosition() / TICKS_PER_REV;
    }

    public void stop() {
        liftMotor.setPower(0);
    }
}
