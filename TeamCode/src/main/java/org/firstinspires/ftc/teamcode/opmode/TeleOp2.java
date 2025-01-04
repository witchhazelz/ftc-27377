package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

// teleop for only the claw
@TeleOp (name = "SimpleClaw")
public class TeleOp2 extends LinearOpMode {
    //store gamepad
    Gamepad currentGamepad1 = new Gamepad();
    Gamepad currentGamepad2 = new Gamepad();

    Gamepad previousGamepad1 = new Gamepad();
    Gamepad previousGamepad2 = new Gamepad();

    // create the object claw
    public Servo claw;

    public static final double OPEN = 0.5;// open
    public static final double CLOSED = 0;//closed
    @Override
    public void runOpMode() {
        // initialialize hardware
        claw = hardwareMap.get(Servo.class,"clawServo");

        // set initial position
        claw.setPosition(CLOSED);
        double position = claw.getPosition();

        // update telemetry
        telemetry.addData("Initialized:Target Position",position);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.x){
                claw.setPosition(OPEN);
                // or claw.setToggleClaw();
            }
            else if (gamepad1.b){
                claw.setPosition(CLOSED);
                // or claw.setClamped();
            }



        }
    }

}
