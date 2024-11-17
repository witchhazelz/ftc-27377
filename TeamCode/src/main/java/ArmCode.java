import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
public class ArmCode {
    private Servo clawServo;

    @Override
    public void runOpMode() {
        // Initialize the servo
        clawServo = hardwareMap.get(Servo.class, "claw_servo");

        // Wait for the start button to be pressed
        waitForStart();

        while (opModeIsActive()) {
            // call method to control claw
            controlClaw(gamepad1);
        }
    }

    private void controlClaw(Gamepad gamepad) {
        // open the claw when the A button is pressed
        if (gamepad.a) {
            clawServo.setPosition(1.0); // adjust value based on servos range
        }
        // close the claw when the B button is pressed
        else if (gamepad.b) {
            clawServo.setPosition(0.0); // adjust value based on servos range
        }
    }

}
