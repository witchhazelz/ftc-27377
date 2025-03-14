package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controls.controllers.PIDController;
import org.firstinspires.ftc.teamcode.controls.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.controls.motion.State;

public class PitchLock {

    private static final double kG = 0.1;
    private static final double MAX_VOLTAGE = 13.0;
    private static final double TICKS_PER_REV = 1440.0;
    private static final double SPROCKET_RADIUS = 1.0;
    private static final double RADIANS_PER_TICK = (2 * Math.PI) / TICKS_PER_REV;
    private static final double INCHES_PER_RADIAN = SPROCKET_RADIUS;

    private final DcMotorEx liftMotor;
    private final VoltageSensor batteryVoltageSensor;
    private final PIDController controller = new PIDController();

    private double targetPositionRadians = 0.0;
    private double currentPositionRadians = 0.0;
    private boolean pitchLocked = false; // Lock flag

    private final ElapsedTime timer = new ElapsedTime();

    public PitchLock(HardwareMap hardwareMap) {
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    public static PIDGains pidGains = new PIDGains(0.005, 0.002, 0, Double.POSITIVE_INFINITY);

    public void moveToPosition(double targetPositionInches) {
        this.targetPositionRadians = targetPositionInches / INCHES_PER_RADIAN;
        pitchLocked = true; // Engage lock when moving
    }

    public void run() {
        currentPositionRadians = liftMotor.getCurrentPosition() * RADIANS_PER_TICK;

        if (pitchLocked) {
            controller.setTarget(new State(targetPositionRadians));
            double pidOutput = controller.calculate(new State(currentPositionRadians));
            double voltageCompensation = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
            double totalOutput = (pidOutput + kG) * voltageCompensation;
            totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));
            liftMotor.setPower(totalOutput);
        }
    }

    public void lockPosition() {
        pitchLocked = true;
        targetPositionRadians = currentPositionRadians; // Maintain current position
    }

    public void unlockPosition() {
        pitchLocked = false;
        liftMotor.setPower(0);
    }

    public double getPosition() {
        return currentPositionRadians * INCHES_PER_RADIAN;
    }

    public void printTelemetry() {
        System.out.println("Target Position (inches): " + (targetPositionRadians * INCHES_PER_RADIAN));
        System.out.println("Current Position (inches): " + getPosition());
        System.out.println("Pitch Locked: " + pitchLocked);
    }
}