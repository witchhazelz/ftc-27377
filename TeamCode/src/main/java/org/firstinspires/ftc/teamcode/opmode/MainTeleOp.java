package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;



import org.firstinspires.ftc.teamcode.subsystems.SimpleDrivetrain;

@TeleOp()
public class MainTeleOp extends OpMode {
    public static GamepadEx gamepadEx1, gamepadEx2;

     SimpleDrivetrain drive = new SimpleDrivetrain();
     IMU imu;

    // public final BulkReader bulkReader;
   // public final MecanumDrive drivetrain;


//    public TeleOp() {
//        //bulkReader = new BulkReader(hardwareMap);
//        //drivetrain = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
//    }


//    @Override
//    public void runOpMode() throws InterruptedException {
//        gamepadEx1 = new GamepadEx(gamepad1);
//        gamepadEx2 = new GamepadEx(gamepad2);
//
//        waitForStart();
//
//        while (opModeIsActive()) {
//
//            bulkReader.bulkRead();
//
//            gamepadEx1.readButtons();
//            gamepadEx2.readButtons();
//
//            drivetrain.setFieldCentricPowers(
//                    new PoseVelocity2d(
//                            new Vector2d(
//                                   gamepadEx1.getLeftY(),
//                                   -gamepadEx1.getLeftX()
//                            ),
//                            -gamepadEx1.getRightX()
//                    )
//            );
//
//
//        }
//    }

    @Override
    public void init() {
        drive.init(hardwareMap);

         imu = hardwareMap.get(IMU.class, "imu");
         RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                 RevHubOrientationOnRobot.LogoFacingDirection.UP,
                 RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
    }
         private void driveFieldRelative(double forward, double right, double rotate) {
         double robotAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
         double theta = Math.atan2(forward, right);
         double r = Math.hypot(forward, right);
         // rotate angle
         theta = AngleUnit.normalizeRadians(theta - robotAngle);

         // convert back to cartesian
         double newForward = r * Math.sin(theta);
         double newRight = r * Math.cos(theta);

             drive.drive(newForward,newRight,rotate);
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;    // forward/backward
        double right = gamepad1.left_stick_x; // left/right strafe with multiplier
        double rotate = gamepad1.right_stick_x;   // rotation

        driveFieldRelative(forward,right,rotate);
    }
}
