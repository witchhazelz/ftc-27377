package org.firstinspires.ftc.teamcode.opmode;

import static org.firstinspires.ftc.teamcode.subsystems.Wrist.BACKWARD;
import static org.firstinspires.ftc.teamcode.subsystems.Wrist.DOWN;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Linear;
import org.firstinspires.ftc.teamcode.subsystems.Pitch;
import org.firstinspires.ftc.teamcode.subsystems.SimpleDrivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.util.SimpleServoPivot;

@TeleOp()
public class MainTeleOp extends LinearOpMode {
    public static GamepadEx gamepadEx1, gamepadEx2;

    public Claw claw;
    public  Wrist wrist;
    public  Pitch pitch;
    Linear linear = new Linear();
    public double pitchPosition = 0.3;
    public double MID = 0.3;
    public double HIGH = 0.8;
    public double FLAT = 0.0;

    SimpleDrivetrain drive = new SimpleDrivetrain();
     IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {

        wrist.init(hardwareMap);
        drive.init(hardwareMap);
        Claw claw = new Claw(hardwareMap);
        linear.LinearSlides(hardwareMap);
        pitch = new Pitch(hardwareMap);



        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);

        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        claw.setClawOpen();//Claw initial position should be open

        pitch.getPosition();// pitch intial position should be 0
        pitchPosition = pitch.getPosition() * ((2 * Math.PI) / 1440.0);// pitch intial position should be 0
        wrist.setDOWN(0); // wrist intitial position should be facing down




        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()){

            double forward = -gamepad1.left_stick_y;    // forward/backward
            double right = gamepad1.left_stick_x; // left/right strafe with multiplier
            double rotate = gamepad1.right_stick_x;   // rotation

            driveFieldRelative(forward,right,rotate);

            // to pick up from set position on the ground
            if (gamepad1.x){
                claw.setClawOpen();
                pitch.moveToPosition(FLAT); // flat position
                linear.moveToPosition(12, 12); // Midway


            }
            // to pick up from chamber
            else if (gamepad2.y){

                pitch.moveToPosition(MID); // Mid position

                linear.moveToPosition(12, 12); // Midway
                sleep(2000);
                pitch.moveToPosition(0.1);
                //sleep(3000);
               // claw.setClawClosed();

            }
            //set to score in baskets
            else if (gamepad1.a){
                claw.setClawClosed();
                sleep(2000);
                pitch.moveToPosition(HIGH);
                linear.moveToPosition(24,24);
                wrist.setBACKWARD(BACKWARD);

            }
            else if (gamepad1.b){
                pitch.moveToPosition(FLAT);
                linear.moveToPosition(0,0);
                wrist.setDOWN(DOWN);
            }

            linear.run();
            pitch.run();


            telemetry.addData("Pitch position", pitchPosition);
        }

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
}
