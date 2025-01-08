package org.firstinspires.ftc.teamcode.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Linear;

@TeleOp()
public class TeleOp6 extends LinearOpMode {

    Linear linear = new Linear();
    @Override
    public void runOpMode() throws InterruptedException {

        linear.LinearSlides(hardwareMap);



        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()){


            if (gamepad1.y){
                linear.runManual(0.2,0.2);

            }
            else if (gamepad1.a){
                linear.runManual(-0.2,-0.2);
            }
            else{
                linear.stop();
            }


            linear.printTelemetry();
            telemetry.addData("left current position",linear.getLeftPosition());
            telemetry.addData("right current position",linear.getRightPosition());


        }

    }

    }
}
