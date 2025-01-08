package org.firstinspires.ftc.teamcode.opmode;


import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Linear; // Ensure this is the correct package for the Linear class

@TeleOp(name = "Claw+Drivetrain+Linear")
public class TeleOp5 extends LinearOpMode {

    // Mecanum drive motors
    private DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private IMU imu;

    // Claw
    private Servo claw;
    private final double OPEN = 0; // open
    private final double CLOSED = 0.5; // closed

    // Linear slides
    private Linear linear = new Linear();

    @Override
    public void runOpMode() {
        // Initialize drivetrain motors
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotorEx.class, "backRightMotor");

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        // Initialize IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        imu.initialize(parameters);

        // Initialize claw
        claw = hardwareMap.get(Servo.class, "clawServo");
        claw.setPosition(CLOSED);

        // Initialize linear slides
        linear.LinearSlides(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Main loop
        while (opModeIsActive()) {
            // --- Drivetrain Control ---
            double y = gamepad1.left_stick_y;    // Forward/backward
            double x = -gamepad1.left_stick_x;   // Left/right strafe
            double rx = -gamepad1.right_stick_x * 0.7;  // Reduced sensitivity for rotation

            if (gamepad1.options) {
                imu.resetYaw();
            }

            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
            double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);
            rotX = rotX * 1.1;

            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // --- Claw Control ---
            if (gamepad1.x) {
                claw.setPosition(OPEN);
            } else if (gamepad1.b) {
                claw.setPosition(CLOSED);
            }

            // --- Linear Slides Control ---
            if (gamepad1.y) {
                linear.runManual(0.2, 0.2);
            } else if (gamepad1.a) {
                linear.runManual(-0.2, -0.2);
            } else {
                linear.stop();
            }

            // Update telemetry
            telemetry.addData("Drive Front Left Power", frontLeftPower);
            telemetry.addData("Drive Front Right Power", frontRightPower);
            telemetry.addData("Drive Back Left Power", backLeftPower);
            telemetry.addData("Drive Back Right Power", backRightPower);
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("Linear Left Position", linear.getLeftPosition());
            telemetry.addData("Linear Right Position", linear.getRightPosition());
            telemetry.update();
        }
    }
}


