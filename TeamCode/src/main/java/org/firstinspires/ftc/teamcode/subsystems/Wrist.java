package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Wrist {

    public Servo leftServo;
    public Servo rightServo;

    public double FORWARD = 0.4;
    public double targetPosition;
    public static double BACKWARD = 0.2;
    public static double DOWN = 0.6;

    public double newRightServoPosition;
    public double newLeftServoPosition;
    public double leftServoPosition = leftServo.getPosition();
    public double rightServoPosition = rightServo.getPosition();

    public void init (HardwareMap hardwareMap){
        leftServo = hardwareMap.get(Servo.class, "leftServo");
        rightServo = hardwareMap.get(Servo.class, "rightServo");

    }

    public void runManual(double position){
        targetPosition=position;
    }

    public void run(){
        leftServo.setPosition(targetPosition);
        rightServo.setPosition(targetPosition);
    }

    public void setPosition(double pos) {
        targetPosition = pos;
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

    public void plusOne(double rightServoPosition,double leftServoPosition){
        newRightServoPosition = rightServoPosition + 0.1;
        newLeftServoPosition = leftServoPosition + 0.1;
    }

    public void minusOne(double rightServoPosition,double leftServoPosition){
        newRightServoPosition = rightServoPosition - 0.1;
        newLeftServoPosition = leftServoPosition - 0.1;
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
