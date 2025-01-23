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
import org.firstinspires.ftc.teamcode.subsystems.Pitch;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOp8 Test")
public class TeleOp9 extends LinearOpMode {

    // Mecanum drive motors
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // Extension system
    // private DcMotorEx leftSlideMotor, rightSlideMotor;
    public Linear linear;
    // public double linearAngle;

    // Pitch system
    private DcMotorEx liftMotor;
    public Pitch pitch;

    // Servo system
    private Servo servo1, servo2;
    private double stop = 0.2;

    // create the object claw
    public Servo claw;

    public static final double OPEN = 0.15;// open
    public static final double CLOSED = 0.5;//closed

    public static final double FORWARD = 0.2;
    public static final double BACKWARD = 0.7;
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

        pitch = new Pitch(hardwareMap);
        linear = new Linear(hardwareMap,imu);

        //leftSlidesInches = linear.getLeftPosition();
        //rightSlidesInches = linear.getRightPosition();

        // Initialize pitch system
        // liftMotor = hardwareMap.get(DcMotorEx.class, "liftMotor");
        // liftMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

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


        //     telemetry.addData("Initialized", rightPosition);
        //    telemetry.addData("Initialized", leftPosition);
        //     telemetry.update();

        double pitchMechanismAngle = 0;

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

            pitchMechanismAngle = (pitch.getMotorRotations() * 360) / 4;

            boolean isVertical = false;

            if (pitchMechanismAngle >20 && pitchMechanismAngle < 40){
                isVertical = true;
            }

            // update the linear angle every loop
            //  linear.updateLinearAngle();
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


            if (gamepad1.dpad_up){
                linear.runManual(0.75,0.75,isVertical);

            }
            else if (gamepad1.dpad_down){
                linear.runManual(-0.4,-0.4,isVertical);
            }
            else{
                linear.stop();
            }


            // Pitch control
            if (gamepad1.left_bumper) {
                pitch.runManual(0.6);
            } else if (gamepad1.right_bumper) {
                pitch.runManual(-0.6);
            } else {
                pitch.runManual(0);
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
            // telemetry.addData("Linear Angle (deg)", linear.linearAngle);
            // telemetry.addData("Is Horizontal", linear.isHorizontal());
            // telemetry.addData("Is Vertical", linear.isVertical());
            // telemetry.addData("Raw IMU Pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            // telemetry.addData("Raw IMU Roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
            // telemetry.addData("Raw IMU Yaw", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.addData("Drive Front Left Power", claw.getPosition());;
            telemetry.addData("Drive Front Right Power", servo1.getPosition());


            telemetry.addData("leftPositionInches", linear.getLeftPosition());
            telemetry.addData("rightPositionInches", linear.getRightPosition());
            telemetry.addData("Pitch Motor Ticks", pitch.getCurrentPosition());
            telemetry.addData("Pitch sprocket angle", pitchMechanismAngle);

            //  telemetry.addData("Angle from Vertical", linear.getCurrentAngle());
            // telemetry.addData("Is Vertical", linear.isVertical());
            //telemetry.addData("IMU Pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            telemetry.addData("Pitch Mechanism Angle", pitch.getMotorRotations() * 360.0);
            //telemetry.addData("Combined Linear Angle", linear.linearAngle);
            // telemetry.update();
            // linear.printDetailedTelemetry();  // This will give us more debug info
            telemetry.update();
        }
    }

}
