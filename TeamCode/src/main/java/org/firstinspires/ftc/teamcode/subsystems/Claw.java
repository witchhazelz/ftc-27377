package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;

public class Claw {


    public static final double CLAW_OPEN_POSITION = 1.0;
    public static final double CLAW_CLOSED_POSITION = 0.0;


    private final Servo clawServo;


    public Claw(HardwareMap hardwareMap) {
        this.clawServo = clawServo;
        close();
    }


    public void open() {
        clawServo.setPosition(CLAW_OPEN_POSITION);
    }


    public void close() {
        clawServo.setPosition(CLAW_CLOSED_POSITION);
    }


    public void toggle() {
        if (clawServo.getPosition() == CLAW_CLOSED_POSITION) {
            open();
        } else {
            close();
        }
    }
}
