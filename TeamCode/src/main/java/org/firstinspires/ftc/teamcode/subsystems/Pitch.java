package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controls.controllers.PIDController;
import org.firstinspires.ftc.teamcode.controls.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.controls.motion.State;

public class Pitch {

    // Constants
    private static final double kG = 0.1; // gravity compensation constant
    private static final double MAX_VOLTAGE = 13.0;
    private static final double TICKS_PER_REV = 1440.0; // example value, adjust based on encoder specs
    private static final double SPROCKET_RADIUS = 1.0; // radius in inches
    private static final double RADIANS_PER_TICK = (2 * Math.PI) / TICKS_PER_REV;
    private static final double INCHES_PER_RADIAN = SPROCKET_RADIUS; // linear distance per radian

    // Hardware and controller
    private final DcMotorEx liftMotor;
    private final VoltageSensor batteryVoltageSensor;
    private final PIDController controller = new PIDController();

    // Position tracking
    private double targetPositionRadians = 0.0;

    //do motor encoder thing, set that, then do the math yada yada, then after istantiate in constructor class
    private double currentPositionRadians = 0.0;

    private final ElapsedTime timer = new ElapsedTime();

    public Pitch(HardwareMap hardwareMap) {
        // initialize hardware
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

    }
        // set up PID gains PLEASE CHANGE THESE VALUES OR WE EILL BE INSANELY COOKED NAD I WILL CRY SO MUCH ANF YOU WILL HAVE TO BUY ME
    //STAR OCEAN THE SECOND STORY R IN ORDER FOR ME TO BE OKAY
        public static PIDGains pidGains = new PIDGains(
                0.005,
                0.002,
                0,
                Double.POSITIVE_INFINITY
        );

    public Pitch(DcMotorEx liftMotor, VoltageSensor batteryVoltageSensor) {
        this.liftMotor = liftMotor;
        this.batteryVoltageSensor = batteryVoltageSensor;
    }

    /**
     * move lift to a target position in linear inches
     *
     * @param targetPositionInches target position in linear inches
     */
    public void moveToPosition(double targetPositionInches) {
        // convert linear target position to radians
        this.targetPositionRadians = targetPositionInches / INCHES_PER_RADIAN;
    }

    /**
     * run lift system (should be called repeatedly in a loop)
     */
    public void run() {
        // read current position in radians
        currentPositionRadians = liftMotor.getCurrentPosition() * RADIANS_PER_TICK;

        // set PID target
        controller.setTarget(new State(targetPositionRadians));

        // calculate PID output
        double pidOutput = controller.calculate(new State(currentPositionRadians));

        // apply gravity compensation and voltage scaling
        double voltageCompensation = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        double totalOutput = (pidOutput + kG) * voltageCompensation;

        // clamp motor power between -1.0 and 1.0
        totalOutput = Math.max(-1.0, Math.min(1.0, totalOutput));

        // Apply power to motor
        liftMotor.setPower(totalOutput);
    }

    /**
     * manually control the lift with a specified power
     *
     * @param power Motor power (-1.0 to 1.0)
     */
    public void runManual(double power) {
        double manualPower = scalePowerForLinearMotion(power);
        liftMotor.setPower(manualPower);
    }

    /**
     * scale power based on current position and linear motion constraints
     *
     * @param power Raw power input
     * @return Scaled power
     */
    private double scalePowerForLinearMotion(double power) {
        // example scaling-reduce power as the lift approaches its maximum range
        double maxLinearPosition = 2 * Math.PI * SPROCKET_RADIUS; // maximum linear range in inches
        double scalingFactor = 1.0 - Math.min(1.0, Math.abs((currentPositionRadians * INCHES_PER_RADIAN) / maxLinearPosition));
        return power * scalingFactor;
    }

    /**
     * stop lift motor.
     */
    public void stop() {
        liftMotor.setPower(0.0);
    }

    /**
     * get current position of the lift in linear inches
     *
     * @return current position in linear inches
     */
    public double getPosition() {
        return currentPositionRadians * INCHES_PER_RADIAN;
    }

    /**
     * debug telemetry data
     */
    public void printTelemetry() {
        System.out.println("Target Position (inches): " + (targetPositionRadians * INCHES_PER_RADIAN));
        System.out.println("Current Position (inches): " + getPosition());
    }
}