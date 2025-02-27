package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.subsystems.Linear;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

// this is the code we used for the comp, limits the exstension always

@TeleOp(name = "TeleOp8")
public class TeleOp8 extends LinearOpMode {

    // Mecanum drive motors
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // Extension system
    // private DcMotorEx leftSlideMotor, rightSlideMotor;
    public Linear linear;
    public double linearAngle;

    // Pitch system
    private DcMotorEx liftMotor;

    // Servo system
    private Servo servo1, servo2;
    private double stop = 0.2;

    // create the object claw
    public Servo claw;

    public static final double OPEN = 0.15;// open
    public static final double CLOSED = 0.5;//closed

    public static final double FORWARD = 0.45;
    public static final double BACKWARD = 0.12;

    public static final double DOWN = 0.6;
    private static final double INCHES_PER_TICK = 0.008;


    public double leftSlidesInches;
    public double rightSlidesInches;

    public double rightPosition;
    public Servo hangServoLeft;



    @Override
    public void runOpMode() {
        // Initialize drive motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        // Initialize extension system
        // leftSlideMotor = hardwareMap.get(DcMotorEx.class, "leftSlideMotor");
        // rightSlideMotor = hardwareMap.get(DcMotorEx.class, "rightSlideMotor");

        // leftSlideMotor.setDirection(DcMotorEx.Direction.FORWARD);
        // rightSlideMotor.setDirection(DcMotorEx.Direction.REVERSE);
        linear = new Linear(hardwareMap,imu);
        //leftSlidesInches = linear.getLeftPosition();
        //rightSlidesInches = linear.getRightPosition();

        // Initialize pitch system
        liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        //intitialize claw
        claw = hardwareMap.get(Servo.class,"clawServo");

        // set initial position
        claw.setPosition(CLOSED);
        double position = claw.getPosition();


        // Initialize servos
        servo1 = hardwareMap.get(Servo.class, "leftServo");
        servo2 = hardwareMap.get(Servo.class, "rightServo");
        servo1.setDirection(Servo.Direction.REVERSE);
        // servo1.setPosition(FORWARD);
        // servo2.setPosition(FORWARD);

        //     // hang stuff
        //hangServoRight = hardwareMap.get(Servo.class, "hangServoRight");
        hangServoLeft = hardwareMap.get(Servo.class, "hangServoLeft");
        //     // hangServoRight.setDirection(Servo.Direction.REVERSE);
        //     // initialialize hardware

        //     hangServoRight.setPosition(0.5);
        //     hangServoLeft.setPosition(0.5);
        //     double rightPosition = hangServoRight.getPosition();
        //     //double leftPosition = hangServoRight.getPosition();
        //   //boolean isUpright = true;



        //     telemetry.addData("Initialized", rightPosition);
        //   // telemetry.addData("Initialized", leftPosition);
        //     telemetry.update();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //if (isStopRequested()) return
        waitForStart();

        while (opModeIsActive()) {
            double y = gamepad1.left_stick_y * 0.5;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x * 0.5;//was 0.5

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            frontLeft.setPower((rotY + rotX + rx) / denominator);
            frontRight.setPower((rotY - rotX - rx) / denominator);
            backLeft.setPower((rotY - rotX + rx) / denominator);
            backRight.setPower((rotY + rotX - rx) / denominator);

            if (gamepad1.right_stick_button) {
                imu.resetYaw();
            }

            // update the linear angle every loop
            linear.updateLinearAngle();
            //linear.getLeftPosition();
            //linear.getRightPosition();

            //servo1.setPosition(FORWARD);
            // servo2.setPosition(FORWARD);

            // Wrist control
            if (gamepad1.a) {
                servo1.setPosition(FORWARD);
                servo2.setPosition(FORWARD);
            } else if (gamepad1.y) {
                servo1.setPosition(BACKWARD);
                servo2.setPosition(BACKWARD);
            }
            // else if (gamepad1.left_stick_button){
            //      servo1.setPosition(DOWN);
            //      servo2.setPosition(DOWN);
            //  }

            // Extension control
            // if (gamepad1.dpad_down) {
            //     leftSlideMotor.setPower(0.6);
            //     rightSlideMotor.setPower(0.6);
            // } else if (gamepad1.dpad_up) {
            //     leftSlideMotor.setPower(-0.5);
            //     rightSlideMotor.setPower(-0.5);
            // } else {
            //     leftSlideMotor.setPower(0);
            //     rightSlideMotor.setPower(0);
            // }
            if (gamepad1.dpad_up){
                linear.runManual(0.75,0.75);

            }
            else if (gamepad1.dpad_down){
                linear.runManual(-0.7,-0.7);
            }
            // else if (gamepad1.dpad_left){
            //     linear.hang();
            // }
            else{
                linear.stop();
            }


            // Pitch control
            if (gamepad1.left_bumper) {
                liftMotor.setPower(0.6);
            } else if (gamepad1.right_bumper) {
                liftMotor.setPower(-0.6);
            } else {
                liftMotor.setPower(0);
            }

            //claw control
            if (gamepad1.x ){
                claw.setPosition(OPEN);
                // or claw.setToggleClaw();
            }
            else if (gamepad1.b){
                claw.setPosition(CLOSED);
                // or claw.setClamped();
            }



            // telemetry.addData("LinearLeftPosition" , linearLeftPosition);
            telemetry.addData("current Angle", linear.linearAngle);
            telemetry.addData("Drive Front Left Power", claw.getPosition());;
            telemetry.addData("Drive Front Right Power", servo1.getPosition());


            telemetry.addData("leftPositionInches", linear.getLeftPosition());
            telemetry.addData("rightPositionInches", linear.getRightPosition());
            telemetry.update();
            linear.printTelemetry();
            telemetry.update();
        }
    }

}