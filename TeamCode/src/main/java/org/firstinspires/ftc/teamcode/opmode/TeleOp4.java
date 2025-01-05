package org.firstinspires.ftc.teamcode.opmode;

// EXSTENSION ONLY TELEOP
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.controls.controllers.PIDController;
import org.firstinspires.ftc.teamcode.controls.gainmatrices.PIDGains;
import org.firstinspires.ftc.teamcode.subsystems.Linear;

@TeleOp
public class TeleOp4 extends LinearOpMode {



    Linear linear = new Linear();

    @Override
    public void runOpMode() throws InterruptedException {

        linear.LinearSlides(hardwareMap);

        //double leftTargetPosition = leftTargetPositionInches;
        //double rightTargetPosition = rightTargetPositionInches;

        if (isStopRequested()) return;

        waitForStart();

        while (opModeIsActive()){

            if (gamepad1.dpad_up){
                linear.moveToPosition(10,10);
            }
            else if(gamepad1.dpad_down){
                linear.moveToPosition(0,0);
            }
            else if (gamepad1.y){
                linear.runManual(0.1,0.1);

            }
            else{
                linear.stop();
            }

            linear.run();

            linear.printTelemetry();



        }

    }
}
