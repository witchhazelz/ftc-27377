package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class SimpleDrivetrain {
    private static DcMotorEx frontLeft;
    private static DcMotorEx frontRight;
    private static DcMotorEx backLeft;
    private static DcMotorEx backRight;

    private IMU imu;

    public void init(HardwareMap hardwareMap){
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        frontLeft.setDirection(DcMotorEx.Direction.FORWARD);
        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

    }

    private static void setPowers(double frontLeftPower, double frontRightPower, double backLeftPower, double backRightPower) {
         double maxSpeed = 1.0;
         maxSpeed = Math.max(maxSpeed, Math.abs(frontLeftPower));
         maxSpeed = Math.max(maxSpeed, Math.abs(frontRightPower));
         maxSpeed = Math.max(maxSpeed, Math.abs(backLeftPower));
         maxSpeed = Math.max(maxSpeed, Math.abs(backRightPower));

         frontLeftPower /= maxSpeed;
         frontRightPower /= maxSpeed;
         backLeftPower /= maxSpeed;
         backRightPower /= maxSpeed;

         frontLeft.setPower(frontLeftPower);
         frontRight.setPower(frontRightPower);
         backLeft.setPower(backLeftPower);
         backRight.setPower(backRightPower);
        }

         // Thanks to FTC16072 for sharing this code!!
         public static void drive(double forward, double right, double rotate) {
         double frontLeftPower = forward + right + rotate;
         double frontRightPower = forward - right - rotate;
         double backLeftPower = forward - right + rotate;
         double backRightPower = forward + right - rotate;

        setPowers(frontLeftPower, frontRightPower, backLeftPower, backRightPower);
    }

    public void stop(){
        setPowers(0,0,0,0);
    }

    public void driveFieldRelative (double forward, double right, double rotate){

        double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        // convert to polar
        double theta = Math.atan2(forward, right);
        double r = Math.hypot(forward, right);
        // rotate angle
        theta = AngleUnit.normalizeRadians(theta - robotAngle);

        // convert back to cartesian
        double newForward = r * Math.sin(theta);
        double newRight = r * Math.cos(theta);

        SimpleDrivetrain.drive(newForward, newRight, rotate);
    }



    public void printTelemetry() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }
}
