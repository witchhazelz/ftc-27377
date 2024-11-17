import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Gamepad;
public class TeleOp {
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private Servo clawServo;
    private Servo wristServo;

    // PID controller for claw
    private PIDController clawPID;
    private double targetClawPosition = 0.0; // Target position for the claw (0.0 to 1.0)

    // PID constants
    private static final double Kp = 1.0;
    private static final double Ki = 0.0;
    private static final double Kd = 0.1;

    @Override
    public void runOpMode() {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_motor");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");

        // Initialize the PID controller
        clawPID = new PIDController(Kp, Ki, Kd);

        waitForStart();

        while (opModeIsActive()) {
            controlClaw(gamepad1);
            controlWrist(gamepad1);
            mecanumDrive(gamepad1);
            updateClawPosition(); // Update claw position using PID
            telemetry.update(); // Update telemetry
        }
    }

    private void controlClaw(Gamepad gamepad) {
        // Set target position based on button presses
        if (gamepad.a) {
            targetClawPosition = 1.0; // Open claw
        } else if (gamepad.b) {
            targetClawPosition = 0.0; // Close claw
        }
    }

    private void updateClawPosition() {
        // Get the current position of the claw servo
        double currentClawPosition = clawServo.getPosition();

        // Calculate the output from the PID controller
        double pidOutput = clawPID.calculate(targetClawPosition, currentClawPosition);

        // Set the servo position based on the PID output
        clawServo.setPosition(clamp(currentClawPosition + pidOutput, 0.0, 1.0));
    }

    private void controlWrist(Gamepad gamepad) {
        if (gamepad.left_bumper) {
            wristServo.setPosition(1.0); // Rotate wrist up
        } else if (gamepad.right_bumper) {
            wristServo.setPosition(0.0); // Rotate wrist down
        }
    }

    private void mecanumDrive(Gamepad gamepad) {
        double drive = -gamepad.left_stick_y; // Forward/backward
        double strafe = gamepad.left_stick_x; // Left
    }
}
