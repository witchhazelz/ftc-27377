package org.firstinspires.ftc.teamcode.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Wrist;

@TeleOp(name = "Wrist test")
public class TeleOp10 extends LinearOpMode {
    Wrist wrist = new Wrist();

    @Override
    public void runOpMode() {

        wrist.init(hardwareMap);



        waitForStart();

        while (opModeIsActive()) {

            double joystickInput = gamepad1.left_stick_y;

            if (gamepad1.a){
               wrist.setDOWN(0);


            }
            else if(gamepad1.y){
                wrist.setBACKWARD(0.5);

            }

            else{
                wrist.close();
            }

            wrist.printTelemetry();

        }
    }
}
