package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.util.BulkReader;

public class TeleOp extends LinearOpMode {
    public static GamepadEx gamepadEx1, gamepadEx2;
    public final BulkReader bulkReader;
    public final MecanumDrive drivetrain;

    public TeleOp() {
        bulkReader = new BulkReader(hardwareMap);
        drivetrain = new MecanumDrive(hardwareMap,new Pose2d(0,0,0));
    }


    @Override
    public void runOpMode() throws InterruptedException {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEx2 = new GamepadEx(gamepad2);

        waitForStart();

        while (opModeIsActive()) {

            bulkReader.bulkRead();

            gamepadEx1.readButtons();
            gamepadEx2.readButtons();

            drivetrain.setFieldCentricPowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                   gamepadEx1.getLeftY(),
                                   -gamepadEx1.getLeftX()
                            ),
                            -gamepadEx1.getRightX()
                    )
            );


        }
    }
}
