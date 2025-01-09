package org.firstinspires.ftc.teamcode.opmode;


// EXSTENSION ONLY TELEOP
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.subsystems.Linear;

@TeleOp (name = "Linear")
public class TeleOp4 extends LinearOpMode {



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
//            telemetry.addData("left current position",linear.getLeftPosition());
//            telemetry.addData("right current position",linear.getRightPosition());


        }

    }
}



