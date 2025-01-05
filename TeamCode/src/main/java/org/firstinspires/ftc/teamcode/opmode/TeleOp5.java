package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.subsystems.Linear;
import org.firstinspires.ftc.teamcode.subsystems.SimpleDrivetrain;

@TeleOp (name = "Drivetrain + Claw + Linear")
public class TeleOp5  extends LinearOpMode {

    private double forward;
    private double right;
    private double rotate;

    private IMU imu;
    SimpleDrivetrain drive = new SimpleDrivetrain();
    //Claw claw = new Claw(hardwareMap);
    Linear linear = new Linear();

    //store gamepad
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad previousGamepad1 = new Gamepad();


    // create the object claw
    public Servo claw;

    public static final double OPEN = 0.5;// open
    public static final double CLOSED = 0;//closed
    @Override
    public void runOpMode() throws InterruptedException {
        // initialialize hardware
        claw = hardwareMap.get(Servo.class,"clawServo");
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

        // set initial position
        claw.setPosition(CLOSED);
        double position = claw.getPosition();

        // update telemetry
        telemetry.addData("Initialized:Target Position",position);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            double forward = -currentGamepad1.left_stick_y;
            double right = currentGamepad1.left_stick_x;
            double rotate = currentGamepad1.right_stick_x;

            // ste drive to field centric
            drive.driveFieldRelative(forward, right, rotate);

            // linear code
            if (currentGamepad1.dpad_up){
                linear.moveToPosition(10,10);
            }
            else if(currentGamepad1.dpad_down){
                linear.moveToPosition(0,0);
            }
            else if (currentGamepad1.y){
                linear.runManual(0.1,0.1);

            }
            else{
                linear.stop();
            }

            linear.run();

            // claw code
            if (currentGamepad1.x){
                claw.setPosition(OPEN);
                // or claw.setToggleClaw();
            }
            else if (currentGamepad1.b){
                claw.setPosition(CLOSED);
                // or claw.setClamped();
            }

            // calls method to print telemetery for drivtrain and linear
            drive.printTelemetry();
            linear.printTelemetry();

        }
    }
}
