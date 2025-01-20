package org.firstinspires.ftc.teamcode;
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

public class Linear {
    // Constants
    private static final double INCHES_PER_TICK = 0.008;
    private static final double MAX_VOLTAGE = 13.0;

    /**
     * Maximum safe extension distance for the linear slides when horizontal (in inches).
     * DO NOT MODIFY THIS VALUE WITHOUT TEAM APPROVAL - Exceeding this limit risks damage to the robot.
     */
    private static final double MAX_HORIZONTAL_EXTENSION = 31.0;

    /**
     * Threshold angle (in degrees) to determine if slides are in a horizontal position.
     * Angles below this value are considered "horizontal" for safety limits.
     */
    private static final double HORIZONTAL_ANGLE_THRESHOLD = 15.0;

    // Hardware
    private DcMotorEx leftSlideMotor;
    private DcMotorEx rightSlideMotor;
    private VoltageSensor batteryVoltageSensor;
    private IMU imu;

    // State tracking
    public double leftCurrentPosition = 0.0;
    public double rightCurrentPosition = 0.0;
    public double linearAngle;

    private final ElapsedTime timer = new ElapsedTime();

    /**
     * Initializes the linear slides and IMU
     */
    public void LinearSlides(HardwareMap hardwareMap) {
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

        leftSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Updates the linear slide angle using IMU data
     */
    public void updateLinearAngle() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        linearAngle = orientation.getPitch(AngleUnit.DEGREES);
    }

    /**
     * Returns true if the slides are in a horizontal position
     */
    public boolean isHorizontal() {
        return Math.abs(linearAngle) < HORIZONTAL_ANGLE_THRESHOLD;
    }

    /**
     * Runs manual control of the slides with safety limits enforced.
     * @param leftPower Desired power for left slide (-1.0 to 1.0)
     * @param rightPower Desired power for right slide (-1.0 to 1.0)
     */
    public void runManual(double leftPower, double rightPower) {
        // Get current positions
        leftCurrentPosition = leftSlideMotor.getCurrentPosition() * INCHES_PER_TICK;
        rightCurrentPosition = rightSlideMotor.getCurrentPosition() * INCHES_PER_TICK;
        
        // Check if near horizontal and at max extension
        boolean isNearHorizontal = Math.abs(linearAngle) < HORIZONTAL_ANGLE_THRESHOLD;
        
        // Limit extension when horizontal
        if (isNearHorizontal) {
            if (leftCurrentPosition >= MAX_HORIZONTAL_EXTENSION && leftPower > 0) {
                leftPower = 0;
            }
            if (rightCurrentPosition >= MAX_HORIZONTAL_EXTENSION && rightPower > 0) {
                rightPower = 0;
            }
        }
        
        // Prevent retraction past 0
        if (leftCurrentPosition <= 0 && leftPower < 0) {
            leftPower = 0;
        }
        if (rightCurrentPosition <= 0 && rightPower < 0) {
            rightPower = 0;
        }
        
        // Apply powers to motors
        leftSlideMotor.setPower(leftPower);
        rightSlideMotor.setPower(rightPower);
    }

    public void stop() {
        leftSlideMotor.setPower(0.0);
        rightSlideMotor.setPower(0.0);
    }
    
    public void resetEncoders() {
        leftCurrentPosition = 0;
        rightCurrentPosition = 0;
    }

    public void printTelemetry() {
        System.out.println("Linear slides current angle " + linearAngle);
        System.out.println("Left Current Position: " + leftCurrentPosition + " inches");
        System.out.println("Right Current Position: " + rightCurrentPosition + " inches");
    }
}