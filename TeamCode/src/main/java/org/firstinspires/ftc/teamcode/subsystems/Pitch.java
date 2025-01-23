package org.firstinspires.ftc.teamcode.subsystems;

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
    private static final double TICKS_PER_REV = 1440.0; // example value --adjust based on encoder specs
    private static final double SPROCKET_RADIUS = 2.8; // radius in inches
    private static final double RADIANS_PER_TICK = (2 * Math.PI) / TICKS_PER_REV;
    private static final double INCHES_PER_RADIAN = SPROCKET_RADIUS; // linear distance per radian

    // predefined positions for manual override
    private static final double POSITION_A_RADIANS = Math.PI / 4; // predefined pos A (45 degrees)
    private static final double POSITION_X_RADIANS = Math.PI / 2; // predefined pos X (90 degrees)
    // (keep in radians, search up degrees to radians if needed)
    //hardware+controller
    private final DcMotorEx liftMotor;
    private final VoltageSensor batteryVoltageSensor;

    // PID variables
    private double kP = 0.0025;// how fast im going to the target
    private double kI = 0.0017;// how fast i wanna build up
    private double kD = 0.0001;// slow down when going to fast
    private double targetPositionRadians = 0.0;
    private double currentPositionRadians = 0.0;
    private double errorIntegral = 0.0;
    private double lastError = 0.0;
    public double ticksPerRotation;

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

        liftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        ticksPerRotation = liftMotor.getMotorType().getTicksPerRev();
    }

    public void moveToPositionRadians(double targetPositionRadians) {
        this.targetPositionRadians = targetPositionRadians;
    }

    public void run() {
        currentPositionRadians = liftMotor.getCurrentPosition() * RADIANS_PER_TICK;

        // PID calculations
        double error = targetPositionRadians - currentPositionRadians;
        errorIntegral += error * timer.seconds();
        double errorDerivative = (error - lastError) / timer.seconds();
        lastError = error;

        double pidOutput = (kP * error) + (kI * errorIntegral) + (kD * errorDerivative);

        // // gravity compensation and voltage scaling
        // double voltageCompensation = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        // double totalOutput = (pidOutput + kG) * voltageCompensation;


        // Calculate gravitational force and required power
        double gravitationalForce = MASS * GRAVITY; // in Newtons
        double requiredPower = gravitationalForce * SPROCKET_RADIUS; // in Watts

        // Convert required power to motor power (assuming linear relationship)
        double minPower = requiredPower / MAX_VOLTAGE; // normalize to motor power range

        // clamp output power
        //totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));

        // Gravity compensation and voltage scaling
        double voltageCompensation = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double totalOutput = (pidOutput + kG) * voltageCompensation;

        // clamp output power
        totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));

        // Locking mechanism
        if (Math.abs(totalOutput) <= minPower) {
            // totalOutput = Math.signum(totalOutput) ?- minPower; // apply minimum power to hold position
            totalOutput = (totalOutput < 0)?-minPower:minPower;
        }


        liftMotor.setPower(totalOutput);
    }

    public void controlLift(Gamepad gamepad) {
        boolean leftBumper = gamepad.left_bumper;
        boolean rightBumper = gamepad.right_bumper;
        boolean overrideA = gamepad.a; // use the "A" button for manual override to pos A
        boolean overrideX = gamepad.x; // use the "X" button for manual override to pos X

        if (overrideA) {
            moveToPositionRadians(POSITION_A_RADIANS); // move to predefined pos A
        } else if (overrideX) {
            moveToPositionRadians(POSITION_X_RADIANS); // move to predefined pos X
        } else if (leftBumper) {
            currentPositionRadians = liftMotor.getCurrentPosition() * RADIANS_PER_TICK; // store current pos
            moveToPositionRadians(currentPositionRadians + (Math.PI / 36)); // increment clockwise continuously
        } else if (rightBumper) {
            currentPositionRadians = liftMotor.getCurrentPosition() * RADIANS_PER_TICK; // store current pos
            moveToPositionRadians(currentPositionRadians - (Math.PI / 36)); // decrement counterclockwise continuously
        } else {
            moveToPositionRadians(currentPositionRadians);
            run(); // lock the pos using PID
        }
    }

    public void runManual(double power) {
        liftMotor.setPower(power);
    }

    public double getMotorRotations(){
        return liftMotor.getCurrentPosition() / TICKS_PER_REV;
    }

    public void stop() {
        liftMotor.setPower(0);

    }
    public double getCurrentPosition(){
        return liftMotor.getCurrentPosition();

    }
}
