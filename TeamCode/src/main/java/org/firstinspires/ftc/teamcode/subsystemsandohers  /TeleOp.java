package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

public class TeleOp extends LinearOpMode {
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor armMotor; // Motor for the arm
    private DcMotor intakeMotor; // Motor for the intake
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
        // initialize motors and servos
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_motor");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor"); // Initialize arm motor
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor"); // Initialize intake motor
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");

        // initialize the PID controller
        clawPID = new PIDController(Kp, Ki, Kd);

        // Create GamepadEx instances for better gamepad handling
        GamepadEx gamepad1Ex = new GamepadEx(gamepad1);
        GamepadEx gamepad2Ex = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            controlClaw(gamepad1Ex);
            controlWrist(gamepad1Ex);
            controlArm(gamepad2Ex); // control arm with gamepad 2
            controlIntake(gamepad2Ex); // control intake with gamepad 2
            mecanumDrive(gamepad1Ex);
            updateClawPosition(); // update claw position using PID
            telemetry.update(); // update telemetry
        }
    }

    private void controlClaw(GamepadEx gamepad) {
        // set target position based on button presses
        if (gamepad.getButton(A).isPressed()) {
            targetClawPosition = 1.0; // Open claw
        } else if (gamepad.getButton(B).isPressed()) {
            targetClawPosition = 0.0; // Close claw
        }
    }

    private void updateClawPosition() {
        // get current position of the claw servo
        double currentClawPosition = clawServo.getPosition();

        // calculate output from the PID controller
        double pidOutput = clawPID.calculate(targetClawPosition, currentClawPosition);

        // Set the servo position based on the PID output
        clawServo.setPosition(clamp(currentClawPosition + pidOutput, 0.0, 1.0));
    }

    private void controlWrist(GamepadEx gamepad) {
        if (gamepad.getButton(LEFT_BUMPER).isPressed()) {
            wristServo.setPosition(1.0); // Rotate wrist up
        } else if (gamepad.getButton(RIGHT_BUMPER).isPressed()) {
            wristServo.setPosition (0.0); // Rotate wrist down
        }
    }

    private void controlArm(GamepadEx gamepad) {
        // Control the arm motor with the right stick
        double armPower = -gamepad.getRightStickY(); // Invert for natural control
        armMotor.setPower(armPower);
    }

    private void controlIntake(GamepadEx gamepad) {
        // Control the intake motor with buttons
        if (gamepad.getTrigger(LEFT_TRIGGER).getValue() > 0.5) {
            intakeMotor.setPower(1.0); // Run intake
        } else if (gamepad.getTrigger(RIGHT_TRIGGER).getValue() > 0.5) {
            intakeMotor.setPower(-1.0); // Reverse intake
        } else {
            intakeMotor.setPower(0.0); // Stop intake
        }
    }

    private void mecanumDrive(GamepadEx gamepad) {
        double drive = -gamepad.getLeftStickY(); // Forward/backward
        double strafe = gamepad.getLeftStickX(); // Left/right
        double rotate = gamepad.getRightStickX(); // Rotation

        // calc motor powers for mecanum drive
        double leftFrontPower = drive + strafe + rotate;
        double rightFrontPower = drive - strafe - rotate;
        double leftBackPower = drive - strafe + rotate;
        double rightBackPower = drive + strafe - rotate;

        // normalize motor powers to make them  [-1, 1]
        double maxPower = Math.max(Math.abs(leftFrontPower), Math.max(Math.abs(rightFrontPower), Math.max(Math.abs(leftBackPower), Math.abs(rightBackPower))));
        if (maxPower > 1.0) {
            leftFrontPower /= maxPower;
            rightFrontPower /= maxPower;
            leftBackPower /= maxPower;
            rightBackPower /= maxPower;
        }

        // motor powers
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
