package org.firstinspires.ftc.teamcode.opmode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class TeleOp extends LinearOpMode {
    public static GamepadEx gamepadEx1, gamepadEX2;
    public final BulkRead

    @Override
    public void runOpMode() throws InterruptedException {
        gamepadEx1 = new GamepadEx(gamepad1);
        gamepadEX2 = new GamepadEx(gamepad2);
    }
}
