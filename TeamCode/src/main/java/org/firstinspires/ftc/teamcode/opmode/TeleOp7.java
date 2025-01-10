package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name = "Wrist")
public class TeleOp7 extends LinearOpMode {
    private Servo servo1;
    private Servo servo2;

    public double stop = 0.0;
    private double wristServoPosition;

    @Override
    public void runOpMode() {

        servo1 = hardwareMap.get(Servo.class, "leftServo");
        servo2 = hardwareMap.get(Servo.class, "rightServo");


        servo1.setPosition(stop);
        servo2.setPosition(stop);
        double servo1Position = servo1.getPosition();
        double servo2Position = servo1.getPosition();

        // update telemetry
        telemetry.addData("Initialized:Servo 1 current Position",servo1Position);
        telemetry.addData("Initialized:Servo 2 current Position",servo2Position);

        telemetry.update();


        waitForStart();

        while (opModeIsActive()) {

            double joystickInput = gamepad1.left_stick_y;
            wristServoPosition = servo1Position + joystickInput +1;
            if (gamepad1.a){
                servo1.setPosition(0.25);
                servo2.setPosition(0.25);


            }
            else if(gamepad1.y){
                servo1.setPosition(0.5);
                servo2.setPosition(0.5);

            }
            else{
                servo1.setPosition(stop);
                servo1.setPosition(stop);
            }
//
//            double wristServo1Position = Range.clip(0.5 + joystickInput * 0.5, 0.0, 1.0);
//            double wristServo2Position = Range.clip(0.5 - joystickInput * 0.5, 0.0, 1.0);
            //double wristServo1Position = Range.clip(0.5 + joystickInput * 0.5, 0.0, 1.0);
            //double wristServo2Position = servo1Position;

//            servo1.setPosition(wristServo1Position);
//            servo2.setPosition(wristServo2Position);
            telemetry.addData("Servo 1 current Position",servo1Position);
            telemetry.addData("Servo 2 current Position",servo2Position);


        }
    }
}
