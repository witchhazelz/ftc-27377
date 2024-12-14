package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.util.BulkReader;

public class TeleOp extends LinearOpMode {
    public static GamepadEx gamepadEx1, gamepadEX2;

    @Override
    public void runOpMode() throws InterruptedException {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEX2 = new GamepadEx(gamepad2);
    }
}
