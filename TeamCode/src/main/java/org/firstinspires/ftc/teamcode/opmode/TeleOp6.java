package org.firstinspires.ftc.teamcode.opmode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import org.firstinspires.ftc.teamcode.subsystems.Linear;
import org.firstinspires.ftc.teamcode.subsystems.Pitch;

@TeleOp(name = "Pitch + Extension")
public class TeleOp6 extends LinearOpMode {

    Linear linear = new Linear();
    public Pitch pitch;
    @Override
    public void runOpMode() throws InterruptedException {

        linear.LinearSlides(hardwareMap);
        pitch = new Pitch(hardwareMap);

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

            if (gamepad1.right_bumper){
                pitch.runManual(0.2);

            }
            else if (gamepad1.left_bumper){
                pitch.runManual(-0.2);
            }
            else{
                pitch.stop();
            }


            linear.printTelemetry();
            telemetry.addData("left current position",linear.getLeftPosition());
            telemetry.addData("right current position",linear.getRightPosition());

            pitch.printTelemetry();


        }

    }

}

