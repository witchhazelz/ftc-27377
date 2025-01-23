package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.IMU;


public class Linear {
    // Constants
    private static final double INCHES_PER_TICK = 0.008;
    private static final double MAX_VOLTAGE = 13.0;

    /**
     * Maximum safe extension distance for the linear slides when horizontal (in inches).
     * DO NOT MODIFY THIS VALUE WITHOUT TEAM APPROVAL - Exceeding this limit risks damage to the robot.
     */
    private static final double MAX_HORIZONTAL_EXTENSION = 14.5;

    private static final double MAX_VERTICAL_EXTENSION = 26.5;

    // Constants for pitch angle calculations
    private static final double TICKS_PER_MOTOR_REV = 1440.0;
    private static final double GEAR_RATIO = 4.0;  // 4:1 gear ratio
    private static final double DEGREES_PER_TICK = (360.0 / TICKS_PER_MOTOR_REV) / GEAR_RATIO;

    // Angle thresholds
    private static final double VERTICAL_ANGLE_THRESHOLD = 5.0;    // ±5 degrees from vertical
    private static final double HORIZONTAL_ANGLE_THRESHOLD = 5.0;  // ±5 degrees from horizontal

    // Reference positions (vertical is 0, horizontal is ~90)
    private static final double VERTICAL_POSITION = 0.0;    // Starting position (vertical)
    private static final double HORIZONTAL_POSITION = 90.0; // When slides are horizontal

    // Hardware
    private DcMotorEx leftSlideMotor;
    private DcMotorEx rightSlideMotor;
    private VoltageSensor batteryVoltageSensor;
    private IMU imu;
    //private DcMotorEx pitchMotor;  // Reference to pitch motor

    // State tracking
    public double leftCurrentPosition = 0.0;
    public double rightCurrentPosition = 0.0;
    public double linearAngle;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Constructor that takes an external IMU reference
     */
    public Linear(HardwareMap hardwareMap, IMU imu) {
        //this.pitchMotor = pitchMotor;
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        this.imu = imu;  // Use the IMU passed from TeleOp

        // Reset encoder at startup (when vertical)
        // pitchMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // pitchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Configure motors
        leftSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Updates the linear slide angle using pitch motor encoder
     * 0 degrees = vertical, 90 degrees = horizontal
     */
    public void updateLinearAngle() {
        //double pitchTicks = pitchMotor.getCurrentPosition();
        // linearAngle = pitchTicks * DEGREES_PER_TICK;

        // // Ensure angle stays within reasonable bounds
        // linearAngle = Math.min(Math.max(linearAngle, -10), 100);  // Allow slight over/under travel

        // // Debug output
        // System.out.println("Pitch Motor Ticks: " + pitchTicks);
        // System.out.println("Calculated Angle from Vertical: " + linearAngle);
        // System.out.println("Is Vertical: " + isVertical());
        // System.out.println("Is Horizontal: " + isHorizontal());
    }

    /**
     * Returns true if the slides are in a vertical position (starting position)
     */
    public boolean isVertical() {
        return Math.abs(linearAngle - VERTICAL_POSITION) < VERTICAL_ANGLE_THRESHOLD;
    }

    /**
     * Returns true if the slides are in a horizontal position
     */
    public boolean isHorizontal() {
        return Math.abs(linearAngle - HORIZONTAL_POSITION) < HORIZONTAL_ANGLE_THRESHOLD;
    }

    /**
     * Gets the current angle from vertical in degrees
     */
    public double getCurrentAngle() {
        return linearAngle;
    }

    /**
     * Runs manual control of the slides with safety limits enforced.
     * @param leftPower Desired power for left slide (-1.0 to 1.0)
     * @param rightPower Desired power for right slide (-1.0 to 1.0)
     */
    public void runManual(double leftPower, double rightPower, boolean isVertical) {
        // Get current positions
        leftCurrentPosition = leftSlideMotor.getCurrentPosition() * INCHES_PER_TICK;
        rightCurrentPosition = rightSlideMotor.getCurrentPosition() * INCHES_PER_TICK;

        double maxExtension = MAX_HORIZONTAL_EXTENSION;
        // Check if near horizontal and at max extension
        //boolean isNearHorizontal = Math.abs(linearAngle - HORIZONTAL_POSITION) < HORIZONTAL_ANGLE_THRESHOLD;

        if (isVertical == true){
            maxExtension = MAX_VERTICAL_EXTENSION;
        }

        // Limit extension when horizontal
        //if (isNearHorizontal) {
        if (leftCurrentPosition >= maxExtension && leftPower > 0) {
            leftPower = 0;
        }
        if (rightCurrentPosition >= maxExtension && rightPower > 0) {
            rightPower = 0;
        }
        //}

        // Prevent retraction past 0
        // if (leftCurrentPosition <= 0 && leftPower < 0) {
        //     leftPower = 0;
        // }
        // if (rightCurrentPosition <= 0 && rightPower < 0) {
        //     rightPower = 0;
        // }

        // Apply powers to motors
        leftSlideMotor.setPower(leftPower);
        rightSlideMotor.setPower(rightPower);
    }

    public void stop() {
        leftSlideMotor.setPower(0.0);
        rightSlideMotor.setPower(0.0);
    }

    public void resetEncoders() {
        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftCurrentPosition = 0;
        rightCurrentPosition = 0;
    }

    public void printTelemetry() {
        System.out.println("Linear slides current angle " + linearAngle);
        System.out.println("Left Current Position: " + leftCurrentPosition + " inches");
        System.out.println("Right Current Position: " + rightCurrentPosition + " inches");
    }

    public double getLeftPosition()
    {
        double leftCurrentPosition = leftSlideMotor.getCurrentPosition();
        double leftSlideInches = leftCurrentPosition*INCHES_PER_TICK;
        return leftSlideInches;
    }
    public double getRightPosition()
    {
        double rightCurrentPosition = rightSlideMotor.getCurrentPosition();
        double rightSlideInches = rightCurrentPosition*INCHES_PER_TICK;
        return rightSlideInches;
    }
    public void hang(){
        leftSlideMotor.setPower(0.6);
        rightSlideMotor.setPower(0.6);

    }

    // Add this method to help with debugging
    public void printDetailedTelemetry() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        System.out.println("Raw Pitch: " + orientation.getPitch(AngleUnit.DEGREES));
        System.out.println("Raw Roll: " + orientation.getRoll(AngleUnit.DEGREES));
        System.out.println("Raw Yaw: " + orientation.getYaw(AngleUnit.DEGREES));
        System.out.println("Adjusted Linear Angle: " + linearAngle);
        System.out.println("Is Vertical: " + isVertical());
        System.out.println("Is Horizontal: " + isHorizontal());
    }
}