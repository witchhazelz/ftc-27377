import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
public class ArmCode {
    private Servo clawServo;

    @Override
    public void runOpMode() {
        // initialize servo
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        //wait for start button
        waitForStart();

        while (opModeIsActive()) {
            // call method to control claw
            controlClaw(gamepad1);
        }
    }

    private void controlClaw(Gamepad gamepad) {
        // open claw when the A button is pressed
        if (gamepad.a) {
            clawServo.setPosition(1.0); // adjust value based on servos range
        }
        // close claw when the B button is pressed
        else if (gamepad.b) {
            clawServo.setPosition(0.0); // adjust value based on servos range
        }
    }

}
