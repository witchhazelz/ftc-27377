import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
public class RotationalWrist {
    private Servo clawServo;
    private Servo wristServo;

    @Override
    public void runOpMode() {
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");

        waitForStart();

        while (opModeIsActive()) {
            controlClaw(gamepad1);
            controlWrist(gamepad1);
            telemetry.update(); // Update telemetry
        }
    }

    private void controlClaw(Gamepad gamepad) {
        if (gamepad.a) {
            clawServo.setPosition(1.0); // Open claw
        } else if (gamepad.b) {
            clawServo.setPosition(0.0); // Close claw
        }
    }

    private void controlWrist(Gamepad gamepad) {
        if (gamepad.left_bumper) {
            wristServo.setPosition(1.0); // Rotate wrist up
        } else if (gamepad.right_bumper) {
            wristServo.setPosition(0.0); // Rotate wrist down
        }
    }
}
