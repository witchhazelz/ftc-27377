package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;



public class Linear {
    // PID variables
    private double kP = 0.005; // Proportional gain
    private double kI = 0.4; // Integral gain
    private double kD = 0.01;   // Derivative gain
    private double targetPositionRadians = 0.0;
    private double currentPositionRadians = 0.0;
    private double errorIntegral = 0.0;
    private double lastError = 0.0;

    // constants
  //  public static PIDGains pidGains = new PIDGains(0.5, 0.4, 0.01, 1.0);
    private static final double kG = 0.1;
    private static final double MAX_VOLTAGE = 13.0;
    private static final double INCHES_PER_TICK = 0.008;

    //hardware and controller
    private DcMotorEx leftSlideMotor;
    private DcMotorEx rightSlideMotor;
    private VoltageSensor batteryVoltageSensor;
//    private final PIDController leftController = new PIDController();
//    private final PIDController rightController = new PIDController();

    // more constants
    private double leftTargetPosition = 0.0;
    private double rightTargetPosition = 0.0;
    private double leftCurrentPosition  ;
    private double rightCurrentPosition  ;
    private double leftError;
    private double rightError;
    private double manualLeftPower = 0.0;
    private double manualRightPower = 0.0;

    public double MAX_EXTENSION_INCHES;

    private final ElapsedTime timer = new ElapsedTime();

    public void LinearSlides(HardwareMap hardwareMap) {

        //initilaization of motors
        leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

       // encoder = hardwareMap.get(DcMotorEx.encoder,)

        leftSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);

        leftSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void moveToPosition(double leftTargetPositionInches, double rightTargetPositionInches) {
        this.leftTargetPosition = leftTargetPositionInches;
        this.rightTargetPosition = rightTargetPositionInches;
    }

    public void run() {
        leftCurrentPosition = leftSlideMotor.getCurrentPosition() * INCHES_PER_TICK;
        rightCurrentPosition = rightSlideMotor.getCurrentPosition() * INCHES_PER_TICK;


        // calculate PID output for left side
        double leftCurrentPosition1 = leftCurrentPosition;
        double leftError = leftTargetPosition - leftCurrentPosition1;

        errorIntegral += leftError * timer.seconds();
        double leftErrorDerivative = (leftError - lastError) / timer.seconds();
        lastError = leftError;

        double leftPidOutput = (kP * leftError) + (kI * errorIntegral) + (kD * leftErrorDerivative);

        // calculate PID output for right
        double rightError = rightTargetPosition - rightCurrentPosition;

        errorIntegral += rightError * timer.seconds();
        double rightErrorDerivative = (rightError - lastError) / timer.seconds();
        lastError = leftError;


        double rightPidOutput = (kP * rightError) + (kI * errorIntegral) + (kD * rightErrorDerivative);

        // apply gravity compensation and voltage scaling
        // voltage compensation
        double voltageCompensation = MAX_VOLTAGE / batteryVoltageSensor.getVoltage();

        double leftTotalOutput = (leftPidOutput + kG) * voltageCompensation;
        double rightTotalOutput = (rightPidOutput + kG) * voltageCompensation;


        leftTotalOutput = Math.max(-1.0, Math.min(1.0, leftTotalOutput));
        rightTotalOutput = Math.max(-1.0, Math.min(1.0, rightTotalOutput));

        leftSlideMotor.setPower(leftTotalOutput);
        rightSlideMotor.setPower(rightTotalOutput);
    }

    public void runManual(double leftPower, double rightPower) {
        double leftPositionInches = leftSlideMotor.getCurrentPosition() * INCHES_PER_TICK;
        double rightPositionInches = rightSlideMotor.getCurrentPosition() * INCHES_PER_TICK;

        if ((leftPower > 0 && leftPositionInches >= MAX_EXTENSION_INCHES) ||
                (leftPower < 0 && leftPositionInches <= 0)) {
            leftPower = 0;
        }

        if ((rightPower > 0 && rightPositionInches >= MAX_EXTENSION_INCHES) ||
                (rightPower < 0 && rightPositionInches <= 0)) {
            rightPower = 0;
        }


        this.manualLeftPower = leftPower;
        this.manualRightPower = rightPower;
        leftSlideMotor.setPower(manualLeftPower);
        rightSlideMotor.setPower(manualRightPower);
    }

    public void stop() {
        leftSlideMotor.setPower(0.0);
        rightSlideMotor.setPower(0.0);
    }

    public double getLeftPosition() {
        return leftCurrentPosition;
    }

    public double getRightPosition() {
        return rightCurrentPosition;
    }

   // public double isEx

    public void printTelemetry() {
        System.out.println("Left Target Position: " + leftTargetPosition + " inches");
        System.out.println("Left Current Position: " + leftCurrentPosition + " inches");
        System.out.println("Right Target Position: " + rightTargetPosition + " inches");
        System.out.println("Right Current Position: " + rightCurrentPosition + " inches");
    }
}

 