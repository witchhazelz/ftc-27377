package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.SimpleDrivetrain;

// TeleOp for both drivetrain and claw
// THIS NEEDS FIXING
@TeleOp
public class TeleOp3 extends LinearOpMode {
    private double forward;
    private double right;
    private double rotate;

    private IMU imu;
    SimpleDrivetrain drive = new SimpleDrivetrain();
    Claw claw = new Claw(hardwareMap);

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);
        //claw.Claw(hardwareMap);
        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        // gamepad state variables
        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();




        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            // store gamepad values from previous loop iteration
            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

             double forward = -currentGamepad1.left_stick_y;
             double right = currentGamepad1.left_stick_x;
             double rotate = currentGamepad1.right_stick_x;

            drive.driveFieldRelative(forward, right, rotate);

            if (gamepad1.x){
                claw.toggleClaw();
            }
            else if (gamepad1.b){
                claw.setClamped(true);
                // or claw.close();
            }


        }

    }
}
