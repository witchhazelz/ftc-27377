package org.firstinspires.ftc.teamcode.opmodes;

import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.A;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.B;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_DOWN;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_LEFT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_RIGHT;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.DPAD_UP;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.LEFT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Button.RIGHT_BUMPER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.LEFT_TRIGGER;
import static com.arcrobotics.ftclib.gamepad.GamepadKeys.Trigger.RIGHT_TRIGGER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public final class MainTeleOp extends LinearOpMode {

    enum TeleOpConfig {
        EDITING_ALLIANCE,
        EDITING_SLOW_LOCK,
        EDITING_FIELD_CENTRIC;

        public static final TeleOpConfig[] selections = values();

        public TeleOpConfig plus(int i) {
            return selections[(ordinal() + i + selections.length) % selections.length];
        }

        public String markIf(TeleOpConfig s) {
            return this == s ? " <" : "";
        }
    }

    private TeleOpConfig selection = TeleOpConfig.EDITING_ALLIANCE;
    private boolean slowModeLocked = false;
    private boolean useFieldCentric = true;

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor armMotor;
    private DcMotor intakeMotor;
    private Servo clawServo;
    private Servo wristServo;

    private PIDController clawPID;
    private double targetClawPosition = 0.0; // target position for claw (0.0 to 1.0)
    private static final double Kp = 1.0;
    private static final double Ki = 0.0;
    private static final double Kd = 0.1;

    private ElapsedTime loopTimer = new ElapsedTime();
    private MultipleTelemetry mTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        // initialize telemetry
        mTelemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // initialize motors and servos
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_motor");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front_motor");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back_motor");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_motor");
        armMotor = hardwareMap.get(DcMotor.class, "arm_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        clawServo = hardwareMap.get(Servo.class, "claw_servo");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");

        // initialize PID controller for claw
        clawPID = new PIDController(Kp, Ki, Kd);

        //GamepadEx instances
        GamepadEx gamepadEx1 = new GamepadEx(gamepad1);
        GamepadEx gamepadEx2 = new GamepadEx(gamepad2);

        // initialization loop
        while (opModeInInit()) {
            gamepadEx1.readButtons();

            if (gamepadEx1.wasJustPressed(DPAD_UP)) selection = selection.plus(-1);
            if (gamepadEx1.wasJustPressed(DPAD_DOWN)) selection = selection.plus(1);

            if (gamepadEx1.wasJustPressed(A)) {
                slowModeLocked = !slowModeLocked;
            }
            if (gamepad Ex1.wasJustPressed(B)) {
                useFieldCentric = !useFieldCentric;
            }

            mTelemetry.addLine("Slow mode " + (slowModeLocked ? "LOCKED" : "unlocked") + selection.markIf(TeleOpConfig.EDITING_SLOW_LOCK));
            mTelemetry.update();
        }

        // control loop
        while (opModeIsActive()) {
            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            // drive control
            double leftX = gamepadEx1.getLeftX();
            double leftY = gamepadEx1.getLeftY();
            double rightX = gamepadEx1.getRightX();

            // arm control
            if (gamepadEx1.isDown(LEFT_BUMPER)) {
                armMotor.setPower(gamepadEx1.getLeftY());
            } else {
                armMotor.setPower(0);
            }

            // claw control with PID
            if (gamepadEx1.wasJustPressed(A)) {
                targetClawPosition = targetClawPosition == 0.0 ? 1.0 : 0.0; // Toggle claw position
            }
            double clawOutput = clawPID.calculate(clawServo.getPosition(), targetClawPosition);
            clawServo.setPosition(clawOutput);

            // wrist control
            if (gamepadEx1.wasJustPressed(DPAD_LEFT)) {
                wristServo.setPosition(0.0); // move wrist to position 0
            } else if (gamepadEx1.wasJustPressed(DPAD_RIGHT)) {
                wristServo.setPosition(1.0); // move wrist to position 1
            }

            // intake control
            intakeMotor.setPower(gamepadEx1.getTrigger(RIGHT_TRIGGER) - gamepadEx1.getTrigger(LEFT_TRIGGER));

            // field-centric driving (maybe)
            if (useFieldCentric) {
                // implement field-centric logic here
            }

            // update telemetry
            mTelemetry.addData("Claw Position", clawServo.getPosition());
            mTelemetry.addData("Arm Power", armMotor.getPower());
            mTelemetry.addData("Wrist Position", wristServo.getPosition());
            mTelemetry.update();

            loopTimer.reset();
        }
    }
}
