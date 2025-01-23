package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.AngleUnit;
//import org.firstinspires.ftc.teamcode.subsystems.Pitch;


public class Linear {
    // PID variables
    private double kP = 0.005; // Proportional gain
    private double kI = 0.04;  // Integral gain
    private double kD = 0.01;  // Derivative gain
    private double errorIntegralLeft = 0.0;
    private double errorIntegralRight = 0.0;
    private double lastErrorLeft = 0.0;
    private double lastErrorRight = 0.0;
    private double maxIntegralSum = 1.0; // Prevent integral windup
    private static final double dt = 0.02; // 20ms loop time

    // constants
    //  public static PIDGains pidGains = new PIDGains(0.5, 0.4, 0.01, 1.0);
    private static final double kG = 0.1;
    private static final double MAX_VOLTAGE = 13.0;
    private static final double INCHES_PER_TICK = 0.008;

    //hardware and controller
    private DcMotorEx leftSlideMotor;
    private DcMotorEx rightSlideMotor;
    private VoltageSensor batteryVoltageSensor;

    // more constants
    public double leftTargetPosition = 0.0;
    public double rightTargetPosition = 0.0;
    public double leftCurrentPosition = 0.0;
    public double rightCurrentPosition = 0.0;
    public double linearAngle;
    private double leftError;
    private double rightError;
    private double manualLeftPower = 0.0;
    private double manualRightPower = 0.0;
    public double MAX_EXTENSION_INCHES = 30;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Maximum safe extension distance for the linear slides when horizontal (in inches).
     * DO NOT MODIFY THIS VALUE WITHOUT TEAM APPROVAL - Exceeding this limit risks damage to the robot.
     */
    private static final double MAX_HORIZONTAL_EXTENSION = -5.5;

    /**
     * Threshold angle (in degrees) to determine if slides are in a horizontal position.
     * Angles below this value are considered "horizontal" for safety limits.
     */
    private static final double HORIZONTAL_ANGLE_THRESHOLD = 15.0;

    /**
     * IMU sensor for measuring slide angle
     */
    private IMU imu;

    /**
     * Initializes the linear slides and IMU
     */
    public void LinearSlides(HardwareMap hardwareMap) {
        //initilaization of motors
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);

        // leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //        leftController.setGains(pidGains);
        //        rightController.setGains(pidGains);
    }

    /**
     * Updates the linear slide angle using IMU data
     * Should be called regularly in the OpMode loop
     */
    public void updateLinearAngle() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        // Pitch gives us the forward/backward tilt
        // Adjust based on your IMU mounting orientation
        linearAngle = orientation.getPitch(AngleUnit.DEGREES);
    }

    /**
     * Returns true if the slides are in a horizontal position
     */
    public boolean isHorizontal() {
        return Math.abs(linearAngle) < HORIZONTAL_ANGLE_THRESHOLD;
    }

    public void run() {
        // Get current positions in inches
        leftCurrentPosition = leftSlideMotor.getCurrentPosition() * INCHES_PER_TICK;
        rightCurrentPosition = rightSlideMotor.getCurrentPosition() * INCHES_PER_TICK;

        // Calculate errors
        leftError = leftTargetPosition - leftCurrentPosition;
        rightError = rightTargetPosition - rightCurrentPosition;

        // Calculate integral terms with anti-windup
        errorIntegralLeft += leftError * dt;
        errorIntegralRight += rightError * dt;
        errorIntegralLeft = Math.min(Math.max(errorIntegralLeft, -maxIntegralSum), maxIntegralSum);
        errorIntegralRight = Math.min(Math.max(errorIntegralRight, -maxIntegralSum), maxIntegralSum);

        // Calculate derivative terms
        double errorDerivativeLeft = (leftError - lastErrorLeft) / dt;
        double errorDerivativeRight = (rightError - lastErrorRight) / dt;

        // Calculate PID outputs
        double leftOutput = kP * leftError +
                kI * errorIntegralLeft +
                kD * errorDerivativeLeft;
        double rightOutput = kP * rightError +
                kI * errorIntegralRight +
                kD * errorDerivativeRight;

        // Add feedforward compensation for gravity
        double gravityCompensation = kG * Math.sin(Math.toRadians(linearAngle));
        leftOutput += gravityCompensation;
        rightOutput += gravityCompensation;

        // Normalize outputs to battery voltage
        double batteryVoltageScaling = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();
        leftOutput *= batteryVoltageScaling;
        rightOutput *= batteryVoltageScaling;

        // Clamp outputs to valid range [-1, 1]
        leftOutput = Math.min(Math.max(leftOutput, -1.0), 1.0);
        rightOutput = Math.min(Math.max(rightOutput, -1.0), 1.0);

        // Store last errors for next iteration
        lastErrorLeft = leftError;
        lastErrorRight = rightError;

        // Apply power to motors
        leftSlideMotor.setPower(leftOutput);
        rightSlideMotor.setPower(rightOutput);
    }

    // Add method to check if slides are at target position
    public boolean isAtTarget() {
        double positionTolerance = 0.5; // inches
        return Math.abs(leftError) < positionTolerance &&
                Math.abs(rightError) < positionTolerance;
    }

    // Add method to tune PID gains
    public void setPIDGains(double p, double i, double d) {
        this.kP = p;
        this.kI = i;
        this.kD = d;
    }

    public void stop() {
        leftSlideMotor.setPower(0.0);
        rightSlideMotor.setPower(0.0);
    }
    public void findLinearAngle() {
        int encoderCounts = leftSlideMotor.getCurrentPosition();  // current encoder position
        int countsPerRevolution = 1440;// number of ticks per revolution
        double linearAngle = (encoderCounts / (double) countsPerRevolution) * 360.0;

        //telemetry.addData("current Angle", angle);
        //telemetry.update();

    }


    // public double getLeftPosition() {
    //     //leftSlideInches = leftCurrentPosition *INCHES_PER_TICK;
    //     leftCurrentPosition = leftSlideMotor.getCurrentPosition();
    //     return leftCurrentPosition;
    // }


    // public double getRightPosition() {
    //   // rightSlideInches = rightCurrentPosition *INCHES_PER_TICK;
    //   rightCurrentPosition = rightSlideMotor.getCurrentPosition();
    //     return rightCurrentPosition;
    // }

    public void resetEncoders(){
        leftCurrentPosition = 0;
        rightCurrentPosition = 0;

    }

    public void printTelemetry() {
        System.out.println("Left Target Position: " + leftTargetPosition + " inches");
        System.out.println("Linear slides current angle " + linearAngle);
        System.out.println("Left Current Position: " + leftCurrentPosition + " inches");
        System.out.println("Right Target Position: " + rightTargetPosition + " inches");
        System.out.println("Right Current Position: " + rightCurrentPosition + " inches");
    }
}
 