package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    public Servo leftServo;
    public Servo rightServo;

    public double FORWARD = 0.5;
    public double BACKWARD = 1;
    public double DOWN = 0;

    public double leftServoPosition = leftServo.getPosition();
    public double rightServoPosition = rightServo.getPosition();

    public void WristServo (HardwareMap hardwareMap){
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

    }

    public void runManual(double position){
        leftServo.setPosition(position);
        rightServo.setPosition(position);

    }

    public void setFORWARD (double FORWARD){
        leftServo.setPosition(FORWARD);
        rightServo.setPosition(FORWARD);

    }

    public void setBACKWARD (double BACKWARD){
        leftServo.setPosition(BACKWARD);
        rightServo.setPosition(BACKWARD);

    }

    public void setDOWN (double DOWN){
        leftServo.setPosition(DOWN);
        rightServo.setPosition(DOWN);

    }

    public void close(){
        leftServo.setPosition(0);
        rightServo.setPosition(0);
    }

    public void printTelemetry(){
        System.out.println("Left current Position: " + leftServoPosition );
        System.out.println("right Current Position: " + rightServoPosition );
    }
}
